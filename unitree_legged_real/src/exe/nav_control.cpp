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
#include <nav_msgs/Odometry.h>

#define N_MOTORS 12

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

Custom custom;

ros::Subscriber sub_cmd_vel;
ros::Publisher pub_joint_state;
ros::Publisher pub_imu;
ros::Publisher pub_odom;

sensor_msgs::Imu imu_msg;
sensor_msgs::JointState joint_state_msg;
nav_msgs::Odometry odom_msg;

/** @brief Map Aliengo internal joint indices to WoLF joints order */
std::array<unsigned int, 12> go1_motor_idxs
        {{
        UNITREE_LEGGED_SDK::FL_0, UNITREE_LEGGED_SDK::FL_1, UNITREE_LEGGED_SDK::FL_2, // LF
        UNITREE_LEGGED_SDK::RL_0, UNITREE_LEGGED_SDK::RL_1, UNITREE_LEGGED_SDK::RL_2, // LH
        UNITREE_LEGGED_SDK::FR_0, UNITREE_LEGGED_SDK::FR_1, UNITREE_LEGGED_SDK::FR_2, // RF
        UNITREE_LEGGED_SDK::RR_0, UNITREE_LEGGED_SDK::RR_1, UNITREE_LEGGED_SDK::RR_2, // RH
        }};
std::array<std::string, 12> go1_motor_names
        {{
        "lf_haa_joint", "lf_hfe_joint", "lf_kfe_joint", // LF
        "lh_haa_joint", "lh_hfe_joint", "lh_kfe_joint", // LH
        "rf_haa_joint", "rf_hfe_joint", "rf_kfe_joint", // RF
        "rh_haa_joint", "rh_hfe_joint", "rh_kfe_joint", // RH
        }};

long cmd_vel_count = 0;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
    printf("cmdVelCallback is running!\t%ld\n", cmd_vel_count);

    custom.high_cmd = rosMsg2Cmd(msg);

    printf("cmd_x_vel = %f\n", custom.high_cmd.velocity[0]);
    printf("cmd_y_vel = %f\n", custom.high_cmd.velocity[1]);
    printf("cmd_yaw_vel = %f\n", custom.high_cmd.yawSpeed);

}

void pubState()
{

    ros::Time t = ros::Time::now();

    joint_state_msg.header.seq            ++;
    joint_state_msg.header.stamp          = t;
    joint_state_msg.header.frame_id       = "trunk";

    for (unsigned int motor_id = 0; motor_id < N_MOTORS; ++motor_id)
    {
        joint_state_msg.name[motor_id]     = go1_motor_names[motor_id];
        joint_state_msg.position[motor_id] = static_cast<double>(custom.high_state.motorState[go1_motor_idxs[motor_id]].q);
        joint_state_msg.velocity[motor_id] = static_cast<double>(custom.high_state.motorState[go1_motor_idxs[motor_id]].dq); // NOTE: this order is different than google
        joint_state_msg.effort[motor_id]   = static_cast<double>(custom.high_state.motorState[go1_motor_idxs[motor_id]].tauEst);
    }

    imu_msg.header.seq            ++;
    imu_msg.header.stamp          = t;
    imu_msg.header.frame_id       = "trunk_imu";
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

    odom_msg.header.seq                 ++;
    odom_msg.header.stamp               = t;
    odom_msg.header.frame_id            = "base";
    odom_msg.child_frame_id             = "odom";
    odom_msg.pose.pose.position.x       = static_cast<double>(custom.high_state.position[0]);
    odom_msg.pose.pose.position.y       = static_cast<double>(custom.high_state.position[1]);
    odom_msg.pose.pose.position.z       = static_cast<double>(custom.high_state.position[2]);
    odom_msg.pose.pose.orientation      = imu_msg.orientation;
    odom_msg.twist.twist.linear.x       = static_cast<double>(custom.high_state.velocity[0]);
    odom_msg.twist.twist.linear.y       = static_cast<double>(custom.high_state.velocity[1]);
    odom_msg.twist.twist.linear.z       = static_cast<double>(custom.high_state.velocity[2]);
    odom_msg.twist.twist.angular        = imu_msg.angular_velocity;
    // Missing Covariance

    pub_odom.publish(odom_msg);
    pub_joint_state.publish(joint_state_msg);
    pub_imu.publish(imu_msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "nav_control");

    ros::NodeHandle nh("go1");

    joint_state_msg.name.resize(N_MOTORS);
    joint_state_msg.position.resize(N_MOTORS);
    joint_state_msg.velocity.resize(N_MOTORS);
    joint_state_msg.effort.resize(N_MOTORS);

    pub_joint_state = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    pub_imu = nh.advertise<sensor_msgs::Imu>("imu", 1);
    pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 1);


    sub_cmd_vel = nh.subscribe("cmd_vel", 1, cmdVelCallback);

    LoopFunc loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&Custom::highUdpSend, &custom));
    LoopFunc loop_udpRecv("high_udp_recv", 0.002, 3, boost::bind(&Custom::highUdpRecv, &custom));
    LoopFunc loop_state("pubState", 0.002, 3, &pubState);

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_state.start();

    ros::spin();

    return 0;
}
