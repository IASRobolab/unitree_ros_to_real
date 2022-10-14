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
          high_udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState))
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

sensor_msgs::Imu imu_msg;
sensor_msgs::JointState joint_state_msg;

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
    for (int motor_id = 0; motor_id < N_MOTORS; ++motor_id)
    {
        joint_state_msg.position[motor_id] = custom.high_state.motorState[motor_id].q;
        joint_state_msg.velocity[motor_id] = custom.high_state.motorState[motor_id].dq; // NOTE: this order is different than google
        joint_state_msg.effort[motor_id]   = custom.high_state.motorState[motor_id].tauEst;
    }

    imu_msg.orientation.w = custom.high_state.imu.quaternion[0];
    imu_msg.orientation.x = custom.high_state.imu.quaternion[1];
    imu_msg.orientation.y = custom.high_state.imu.quaternion[2];
    imu_msg.orientation.z = custom.high_state.imu.quaternion[3];
    imu_msg.angular_velocity.x = custom.high_state.imu.gyroscope[0];
    imu_msg.angular_velocity.y = custom.high_state.imu.gyroscope[1];
    imu_msg.angular_velocity.z = custom.high_state.imu.gyroscope[2];
    imu_msg.linear_acceleration.x = custom.high_state.imu.accelerometer[0];
    imu_msg.linear_acceleration.y = custom.high_state.imu.accelerometer[1];
    imu_msg.linear_acceleration.z = custom.high_state.imu.accelerometer[2];

    pub_joint_state.publish(joint_state_msg);
    pub_imu.publish(imu_msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "nav_control");

    ros::NodeHandle nh("go1");

    joint_state_msg.position.resize(N_MOTORS);
    joint_state_msg.velocity.resize(N_MOTORS);
    joint_state_msg.effort.resize(N_MOTORS);

    pub_joint_state = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    pub_imu = nh.advertise<sensor_msgs::Imu>("imu", 1);

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
