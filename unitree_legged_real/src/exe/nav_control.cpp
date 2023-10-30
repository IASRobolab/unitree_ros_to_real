#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <chrono>
#include <pthread.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <wolf_controller_utils/basefoot_estimator.h>
#include <wolf_controller_utils/tools.h>
#include <tf2_eigen/tf2_eigen.h>
#include <std_srvs/Trigger.h>

#define N_MOTORS 12
#define TRUNK "trunk"
#define IMU "trunk_imu"
#define BASEFOOT "base_footprint"
#define ODOM "odom"

template< class T >
double avg(T* vect, unsigned int size)
{
    T res = 0.0;
    for(unsigned int i=0;i<size;i++)
        res = res + vect[i];
    return res/size;
}

using namespace UNITREE_LEGGED_SDK;
class Custom
{
public:
    UDP low_udp;
    UDP high_udp;

    HighCmd high_cmd = {0};
    HighState high_state = {0};

    LowCmd low_cmd = {0};
    LowState low_state = {0};
    
    xRockerBtnDataStruct keyData;

public:
    Custom()
        : low_udp(LOWLEVEL),
          high_udp(8090, "192.168.12.1", 8082, sizeof(HighCmd), sizeof(HighState))
    {
        high_udp.InitCmdData(high_cmd);
        low_udp.InitCmdData(low_cmd);
    }

    void highUdpSend()
    {
        // printf("high udp send is running\n");

        high_udp.SetSend(high_cmd);
        high_udp.Send();
    }

    void lowUdpSend()
    {

        low_udp.SetSend(low_cmd);
        low_udp.Send();
    }

    void lowUdpRecv()
    {

        low_udp.Recv();
        low_udp.GetRecv(low_state);
    }

    void highUdpRecv()
    {
        // printf("high udp recv is running\n");

        high_udp.Recv();
        high_udp.GetRecv(high_state);
    }
};

static Custom custom;

static ros::Subscriber sub_cmd_vel;
static ros::Subscriber sub_gps;
static ros::Publisher pub_joint_state;
static ros::Publisher pub_imu;
static ros::Publisher pub_odom;
static ros::Publisher pub_battery;
static ros::Publisher pub_gps;
std::shared_ptr<tf2_ros::TransformBroadcaster> pub_tf;
static ros::ServiceServer srv_standup;
static ros::ServiceServer srv_standdown;

static sensor_msgs::Imu imu_msg;
static sensor_msgs::JointState joint_state_msg;
static nav_msgs::Odometry odom_msg;
static sensor_msgs::BatteryState battery_msg;
static sensor_msgs::NavSatFix gps_msg;
static geometry_msgs::TransformStamped odom_T_basefoot;
static geometry_msgs::TransformStamped odom_T_trunk;
static geometry_msgs::TransformStamped basefoot_T_trunk;

static std::vector<bool> contact_states(4,true);
static std::vector<double> contact_heights(4,0.0);

static std::string tf_prefix = "";

ros::Time t;
ros::Time t_prev;
ros::Time t_timer;
bool timer_on = false;

using namespace wolf_controller_utils;
static BasefootEstimator basefoot_estimator;

/** @brief Map Aliengo internal joint indices to WoLF joints order */
static std::array<unsigned int, 12> go1_motor_idxs
        {{
        UNITREE_LEGGED_SDK::FL_0, UNITREE_LEGGED_SDK::FL_1, UNITREE_LEGGED_SDK::FL_2, // LF
        UNITREE_LEGGED_SDK::RL_0, UNITREE_LEGGED_SDK::RL_1, UNITREE_LEGGED_SDK::RL_2, // LH
        UNITREE_LEGGED_SDK::FR_0, UNITREE_LEGGED_SDK::FR_1, UNITREE_LEGGED_SDK::FR_2, // RF
        UNITREE_LEGGED_SDK::RR_0, UNITREE_LEGGED_SDK::RR_1, UNITREE_LEGGED_SDK::RR_2, // RH
        }};
static std::array<std::string, 12> go1_motor_names
        {{
        "lf_haa_joint", "lf_hfe_joint", "lf_kfe_joint", // LF
        "lh_haa_joint", "lh_hfe_joint", "lh_kfe_joint", // LH
        "rf_haa_joint", "rf_hfe_joint", "rf_kfe_joint", // RF
        "rh_haa_joint", "rh_hfe_joint", "rh_kfe_joint", // RH
        }};

static long cmd_vel_count = 0;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    //printf("cmdVelCallback is running!\t%ld\n", cmd_vel_count);
    
    if ( std::abs(custom.keyData.rx) < 0.1 && 
         std::abs(custom.keyData.lx) < 0.1 && 
         std::abs(custom.keyData.ry) < 0.1 && 
         std::abs(custom.keyData.ly) < 0.1 )
    {
        if(timer_on && (t - t_timer).sec >= 6 || !timer_on)
        {
            custom.high_cmd = rosMsg2Cmd(msg);
            printf("**** CMD VEL ****\n");
            printf("cmd_x_vel   = %f\n", custom.high_cmd.velocity[0]);
            printf("cmd_y_vel   = %f\n", custom.high_cmd.velocity[1]);
            printf("cmd_yaw_vel = %f\n", custom.high_cmd.yawSpeed);

            timer_on = false;
        }
    }
    else
    {
        if(!timer_on)
            timer_on = true;
        t_timer = ros::Time::now();
        custom.high_cmd.velocity[0] = 0;
        custom.high_cmd.velocity[1] = 0;
        custom.high_cmd.yawSpeed = 0;
    }
}

void gpsCallback(const sensor_msgs::NavSatFix::Ptr &msg)
{
    msg->header.frame_id = tf_prefix+TRUNK;
    pub_gps.publish(msg);
}

bool standupCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    custom.high_cmd.mode = 6;
    custom.high_cmd.velocity[0] = 0;
    custom.high_cmd.velocity[1] = 0;
    custom.high_cmd.yawSpeed    = 0;
    res.success = true;
    return true;
}

bool standdownCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    custom.high_cmd.mode = 5;
    custom.high_cmd.velocity[0] = 0;
    custom.high_cmd.velocity[1] = 0;
    custom.high_cmd.yawSpeed    = 0;
    res.success = true;
    return true;
}

void pubState()
{

    t = ros::Time::now();

    if(t != t_prev)
    {

//****************** Joint state ******************

	    joint_state_msg.header.seq            ++;
	    joint_state_msg.header.stamp          = t;
	    joint_state_msg.header.frame_id       = tf_prefix+TRUNK;

	    for (unsigned int motor_id = 0; motor_id < N_MOTORS; ++motor_id)
	    {
            joint_state_msg.name[motor_id]     = go1_motor_names[motor_id];
            joint_state_msg.position[motor_id] = static_cast<double>(custom.high_state.motorState[go1_motor_idxs[motor_id]].q);
            joint_state_msg.velocity[motor_id] = static_cast<double>(custom.high_state.motorState[go1_motor_idxs[motor_id]].dq); // NOTE: this order is different than google
            joint_state_msg.effort[motor_id]   = static_cast<double>(custom.high_state.motorState[go1_motor_idxs[motor_id]].tauEst);
	    }

//****************** IMU ******************

	    imu_msg.header.seq            ++;
	    imu_msg.header.stamp          = t;
	    imu_msg.header.frame_id       = tf_prefix+IMU;
	    imu_msg.orientation.w         = static_cast<double>(custom.high_state.imu.quaternion[0]);
	    imu_msg.orientation.x         = static_cast<double>(custom.high_state.imu.quaternion[1]);
	    imu_msg.orientation.y         = static_cast<double>(custom.high_state.imu.quaternion[2]);
	    imu_msg.orientation.z         = static_cast<double>(custom.high_state.imu.quaternion[3]);
	    imu_msg.angular_velocity.x    = static_cast<double>(custom.high_state.imu.gyroscope[0]);
	    imu_msg.angular_velocity.y    = static_cast<double>(custom.high_state.imu.gyroscope[1]);
	    imu_msg.angular_velocity.z    = static_cast<double>(custom.high_state.imu.gyroscope[2]);
	    imu_msg.linear_acceleration.x = static_cast<double>(custom.high_state.imu.accelerometer[0]);
	    imu_msg.linear_acceleration.y = static_cast<double>(custom.high_state.imu.accelerometer[1]);
	    imu_msg.linear_acceleration.z = static_cast<double>(custom.high_state.imu.accelerometer[2]);

//****************** ODOM MSG ******************

	    odom_T_trunk.transform.translation.x       = static_cast<double>(custom.high_state.position[0]);
	    odom_T_trunk.transform.translation.y       = static_cast<double>(custom.high_state.position[1]);
	    odom_T_trunk.transform.translation.z       = static_cast<double>(custom.high_state.position[2]);
	    odom_T_trunk.transform.rotation            = imu_msg.orientation;

	    odom_msg.header.seq                 ++;
	    odom_msg.header.stamp               = t;
	    odom_msg.header.frame_id            = tf_prefix+ODOM;
	    odom_msg.child_frame_id             = tf_prefix+BASEFOOT;
	    odom_msg.pose.pose.position.x       = odom_T_trunk.transform.translation.x;
	    odom_msg.pose.pose.position.y       = odom_T_trunk.transform.translation.y;
	    odom_msg.pose.pose.position.z       = 0.0;
	    odom_msg.pose.pose.orientation      = imu_msg.orientation;
	    odom_msg.twist.twist.linear.x       = static_cast<double>(custom.high_state.velocity[0]);
	    odom_msg.twist.twist.linear.y       = static_cast<double>(custom.high_state.velocity[1]);
	    odom_msg.twist.twist.linear.z       = static_cast<double>(custom.high_state.velocity[2]);
	    odom_msg.twist.twist.angular        = imu_msg.angular_velocity;
	    // TODO: Missing Covariance

//****************** ODOM -> BASEFOOT ******************

	    odom_T_basefoot.header.seq              ++;
	    odom_T_basefoot.header.stamp            = t;
	    odom_T_basefoot.header.frame_id         = tf_prefix+ODOM;
	    odom_T_basefoot.child_frame_id          = tf_prefix+BASEFOOT;
	    odom_T_basefoot.transform.translation.x = odom_msg.pose.pose.position.x;
	    odom_T_basefoot.transform.translation.y = odom_msg.pose.pose.position.y;
	    odom_T_basefoot.transform.translation.z = odom_msg.pose.pose.position.z;
	    odom_T_basefoot.transform.rotation.x    = odom_msg.pose.pose.orientation.x;
	    odom_T_basefoot.transform.rotation.y    = odom_msg.pose.pose.orientation.y;
	    odom_T_basefoot.transform.rotation.z    = odom_msg.pose.pose.orientation.z;
	    odom_T_basefoot.transform.rotation.w    = odom_msg.pose.pose.orientation.w;

	    pub_tf->sendTransform(odom_T_basefoot);

//****************** BASEFOOT -> TRUNK ******************

	    // Define the contact state
	    for(unsigned int i = 0; i< 4; i++)
	    {
	      //printf("force[%i] = %f\n", i, custom.high_state.footForce[i]);
          //printf("height[%i] = %f\n", i, custom.high_state.footPosition2Body[i].z);
	      contact_states[i]  = (std::abs(custom.high_state.footForce[i]) > 0.0 ? true : false );
	      contact_heights[i] = -static_cast<double>(custom.high_state.footPosition2Body[i].z);
	    }
	    basefoot_estimator.setContacts(contact_states,contact_heights);
	    basefoot_estimator.setBasePoseInOdom(tf2::transformToEigen(odom_T_trunk));
	    basefoot_estimator.update();

	    basefoot_T_trunk = tf2::eigenToTransform(basefoot_estimator.getBasefootPoseInBase().inverse());
	    basefoot_T_trunk.header.seq       ++;
	    basefoot_T_trunk.header.stamp    = t;
	    basefoot_T_trunk.header.frame_id = tf_prefix+BASEFOOT;
	    basefoot_T_trunk.child_frame_id  = tf_prefix+TRUNK;

	    pub_tf->sendTransform(basefoot_T_trunk);

//****************** BATTERY STATE ******************

	    battery_msg.header.seq              ++;
	    battery_msg.header.stamp            = t;
	    battery_msg.header.frame_id         = tf_prefix+TRUNK;
	    battery_msg.current                 = static_cast<float>(1000.0 * custom.high_state.bms.current); // mA -> A
	    battery_msg.voltage                 = static_cast<float>(1000.0 * avg<uint16_t>(&custom.high_state.bms.cell_vol[0],10)); // mA -> A
	    battery_msg.percentage              = custom.high_state.bms.SOC;

	    pub_battery.publish(battery_msg);
	    pub_odom.publish(odom_msg);
	    pub_joint_state.publish(joint_state_msg);
	    pub_imu.publish(imu_msg);
	    
//****************** WIRELESS REMOTE CHECK ****************
        memcpy(&custom.keyData, &custom.high_state.wirelessRemote[0], 40);   
        //std::cout << "lx "<< custom.keyData.lx << std::endl;
        //std::cout << "ly "<< custom.keyData.ly << std::endl;
        //std::cout << "rx "<< custom.keyData.rx << std::endl;
        //std::cout << "ry "<< custom.keyData.ry << std::endl;

    }

    t_prev = t;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nav_control");

    //ros::NodeHandle nh("go1");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    priv_nh.getParam("tf_prefix", tf_prefix);
    // Check and fix tf_prefix
    wolf_controller_utils::fixTFprefix(tf_prefix);

    t = t_prev = t_timer = ros::Time::now();

    joint_state_msg.name.resize(N_MOTORS);
    joint_state_msg.position.resize(N_MOTORS);
    joint_state_msg.velocity.resize(N_MOTORS);
    joint_state_msg.effort.resize(N_MOTORS);

    srv_standup   = nh.advertiseService("wolf_controller/stand_up",   standupCallback);
    srv_standdown = nh.advertiseService("wolf_controller/stand_down", standdownCallback);

    pub_joint_state = nh.advertise<sensor_msgs::JointState>("joint_states", 20);
    pub_imu = nh.advertise<sensor_msgs::Imu>("imu", 20);
    pub_odom = nh.advertise<nav_msgs::Odometry>("odometry/robot", 20);
    pub_battery = nh.advertise<sensor_msgs::BatteryState>("battery_state", 20);
    pub_gps = nh.advertise<sensor_msgs::NavSatFix>("gps/fix", 20);
    pub_tf.reset(new tf2_ros::TransformBroadcaster);

    sub_cmd_vel = nh.subscribe("cmd_vel", 20,  cmdVelCallback);
    sub_gps     = nh.subscribe("location", 20, gpsCallback);

    LoopFunc loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&Custom::highUdpSend, &custom));
    LoopFunc loop_udpRecv("high_udp_recv", 0.002, 3, boost::bind(&Custom::highUdpRecv, &custom));
    LoopFunc loop_state("pubState", 0.002, 3, &pubState);

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_state.start();

    ros::spin();

    return 0;
}
