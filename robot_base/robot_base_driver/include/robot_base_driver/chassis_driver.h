#ifndef CHASSIS_DRIVER_H
#define CHASSIS_DRIVER_H

#include <memory>
#include <inttypes.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <csignal>
#include <thread>

#include <iostream>
#include <string.h>
#include <string>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <rcl/types.h>
#include <sys/stat.h>

#include <serial/serial.h>
#include <fcntl.h>
#include <stdbool.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_broadcaster.h>
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "robot_msg/msg/data.hpp"
#include "robot_msg/msg/sign.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
using namespace std;

#define DATA_CHECK        1
#define RECV_CHECK        0
#define FRAME_HEAD        0X7B
#define FRAME_TAIL        0X7D
#define RECV_BUF_SIZE     24
#define SEND_BUF_SIZE     11
#define PI_VAL            3.1415926f

#define GYRO_SCALE        0.00026644f
#define ACCEL_SCALE       1671.84f

extern sensor_msgs::msg::Imu Mpu6050;

const double odom_cov_36[36] = {
    1e-3, 0, 0, 0, 0, 0,
    0, 1e-3, 0, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e3
};

const double odom_cov_alt[36] = {
    1e-9, 0, 0, 0, 0, 0,
    0, 1e-3, 1e-9, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-9
};

const double twist_cov_36[36] = {
    1e-3, 0, 0, 0, 0, 0,
    0, 1e-3, 0, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e3
};

const double twist_cov_alt[36] = {
    1e-9, 0, 0, 0, 0, 0,
    0, 1e-3, 1e-9, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-9
};

typedef struct __PoseVelData {
    float x;
    float y;
    float theta;
} PoseVelData;

typedef struct __ImuRawData {
    short accel_x;
    short accel_y;
    short accel_z;
    short gyro_x;
    short gyro_y;
    short gyro_z;
} ImuRawData;

typedef struct _TxFrame {
    uint8_t bytes[SEND_BUF_SIZE];
    float vx;
    float vy;
    float wz;
    unsigned char tail;
} TxFrame;

typedef struct _RxFrame {
    uint8_t bytes[RECV_BUF_SIZE];
    uint8_t stop_flag;
    unsigned char head;
    float vx;
    float vy;
    float vz;
    float battery;
    unsigned char tail;
} RxFrame;

class ChassisDriver : public rclcpp::Node {
public:
    ChassisDriver();
    ~ChassisDriver();
    void Run();
    void PublishOdometry();

public:
    serial::Serial stm32_port;

private:
    void declare_parameters();
    void load_parameters();

    void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
    void on_ackermann(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);

    void PublishImu();
    void PublishBattery();
    auto createQuaternionMsgFromYaw(double yaw);

    bool ReadSensorData();
    unsigned char ComputeChecksum(unsigned char count, unsigned char mode);
    short DecodeIMU(uint8_t high, uint8_t low);
    float DecodeOdom(uint8_t high, uint8_t low);

    void on_sign_switch(const std_msgs::msg::Int32::SharedPtr msg);

private:
    rclcpp::Time now_, last_;
    float dt_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ack_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr test_pub_;
    rclcpp::Publisher<robot_msg::msg::Data>::SharedPtr pose_pub_;
    rclcpp::Publisher<robot_msg::msg::Data>::SharedPtr vel_pub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_bro_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
    rclcpp::TimerBase::SharedPtr test_timer_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
    rclcpp::TimerBase::SharedPtr imu_timer_;
    rclcpp::TimerBase::SharedPtr battery_timer_;
    rclcpp::TimerBase::SharedPtr pose_timer_;
    rclcpp::TimerBase::SharedPtr vel_timer_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sign_sub_;

    string port_name_, base_frame_, gyro_frame_, odom_frame_, ack_topic_, test_topic_;
    string cmd_topic_;
    int baud_rate_;
    RxFrame rx_data_;
    TxFrame tx_data_;

    PoseVelData pose_;
    PoseVelData vel_;
    ImuRawData imu_raw_;
    float battery_voltage_;
    size_t count_;
};

#endif  // CHASSIS_DRIVER_H