#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <math.h>
#include <unistd.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <sys/time.h>
#include <fstream>
#include <sstream>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/JointState.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include <dynamic_reconfigure/server.h>
#include <base_control/BaseConfig.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>

#include "uart.h"
#include "pid.h"


using namespace std;

enum {
  POWER_SUPPLY_STATUS_UNKNOWN = 0u,
  POWER_SUPPLY_STATUS_CHARGING = 1u,
  POWER_SUPPLY_STATUS_DISCHARGING = 2u,
  POWER_SUPPLY_STATUS_NOT_CHARGING = 3u,
  POWER_SUPPLY_STATUS_FULL = 4u,
  POWER_SUPPLY_HEALTH_UNKNOWN = 0u,
  POWER_SUPPLY_HEALTH_GOOD = 1u,
  POWER_SUPPLY_HEALTH_OVERHEAT = 2u,
  POWER_SUPPLY_HEALTH_DEAD = 3u,
  POWER_SUPPLY_HEALTH_OVERVOLTAGE = 4u,
  POWER_SUPPLY_HEALTH_UNSPEC_FAILURE = 5u,
  POWER_SUPPLY_HEALTH_COLD = 6u,
  POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE = 7u,
  POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE = 8u,
  POWER_SUPPLY_TECHNOLOGY_UNKNOWN = 0u,
  POWER_SUPPLY_TECHNOLOGY_NIMH = 1u,
  POWER_SUPPLY_TECHNOLOGY_LION = 2u,
  POWER_SUPPLY_TECHNOLOGY_LIPO = 3u,
  POWER_SUPPLY_TECHNOLOGY_LIFE = 4u,
  POWER_SUPPLY_TECHNOLOGY_NICD = 5u,
  POWER_SUPPLY_TECHNOLOGY_LIMN = 6u,
};


#pragma pack(1)

typedef struct
{
    unsigned char head1;//数据头1 'S' 0x53
    unsigned char head2;//数据头2 'T' 0x54
    unsigned char struct_size;//结构体长度

    short encoder1;//编码器当前值1
    short encoder2;//编码器当前值2

    short gyro[3];//MPU6050角速度
    short accel[3];//MPU6050线加速度
    short angle_100[3];//通过DMP模块读出来的四元数转的角度

    short vbat_mv;//电池电压 mV
    unsigned char charging;//是否正在充电
    unsigned char full_charged;//是否充满电

    unsigned char asr_id;//语音命令ID

    unsigned char end1;//数据尾1 'U' 0x55
    unsigned char end2;//数据尾2 'V' 0x56
    unsigned char end3;//数据尾3 '\r' 0x0d
    unsigned char end4;//数据尾4 '\n' 0x0a
}McuData;


typedef struct
{
    unsigned char head1;//数据头1 'S' 0x53
    unsigned char head2;//数据头2 'T' 0x54
    unsigned char struct_size;//结构体长度

    short pwm1;//油门PWM1
    short pwm2;//油门PWM2
    unsigned char enable_sound;//是否开启声音

    unsigned char end1;//数据尾1 'U' 0x55
    unsigned char end2;//数据尾2 'V' 0x56
    unsigned char end3;//数据尾3 '\r' 0x0d
    unsigned char end4;//数据尾4 '\n' 0x0a
}CmdData;

#pragma pack()







typedef struct
{
    int wheel_circumference_mm;
    int pulses_per_wheel_turn_around;
    int pulses_per_robot_turn_around;

    int max_pwm;

    int pid_p;
    int pid_i;
    int pid_d;
}SettingData;






class BaseControl
{
public:
    BaseControl();
    ~BaseControl();
    void run();

private:
    void dynamic_reconfigure_callback(base_control::BaseConfig &config, uint32_t level);
    void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
    void pub_battery_msg();
    void pub_joint_msg(double l_angle,double r_angle);
    void control_robot(int target1,int target2);
    void pub_tf_and_odom(ros::Time ros_time_now,double delta_m_s,double delta_rad_s);
    void pub_plot(vector<float> array);


    CmdData cmd_data = {'S','T',sizeof(CmdData),0,0,0,'U','V','\r','\n'};

    double target_m_s=0;//目标线速度m/s
    double target_rad_s=0;//目标角速度rad/s

    ros::NodeHandle nh;

    string odom_frame_id,base_frame_id;
    sensor_msgs::BatteryState battery_msg;

    ros::Subscriber command_sub;
    ros::Publisher odom_pub, battery_pub, tts_pub;
    ros::Publisher joint_pub;
    ros::Publisher plot_pub;
    ros::Publisher enable_tracking_pub;
    ros::Publisher asr_id_pub;

    Uart uart;


    McuData mcu_data;
    unsigned int cnt=0;

    SettingData setting_data;

    double current_time=0,previous_time=0;
    double dt=0,pose_yaw=0;
    double pose_x=0,pose_y=0;

    tf::TransformBroadcaster odom_broadcaster;
    nav_msgs::Odometry odom_msg;
    geometry_msgs::TransformStamped odom_trans;
    geometry_msgs::Quaternion odom_quat;


    string recv_str;
    
    short last_charing=0;
    bool allow_remind_low_power=true;
    
    long long all_encoder1=0;
    long long all_encoder2=0;
    
    dynamic_reconfigure::Server<base_control::BaseConfig> server;
    dynamic_reconfigure::Server<base_control::BaseConfig>::CallbackType dynamic_config;

    PidController left_pid;
    PidController right_pid;

    int last_target1=0;
    int last_target2=0;

    double l_angle=0;
    double r_angle=0;

    double pluses_m = 0;
    double wheel_distance_m = 0;

    unsigned char enable_sound = 1;//默认开启扬声器


};