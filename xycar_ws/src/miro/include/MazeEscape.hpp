// go.hpp
#include <geometry_msgs/Twist.h> 
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <xycar_msgs/xycar_motor.h>
#include "opencv2/opencv.hpp"

#define MAX_ULTRA_RANGE 140
#define AVOID 0
#define NORMAL 1
#define BACK 2
#define PARKING 4
#define OUT 5
#define LEFT_START 126
#define LEFT_END 1
#define RIGHT_START 378
#define RIGHT_END 504
#define STANDBY 100

#define FULL_RIGHT_STEERING 50
#define FULL_LEFT_STEERING -50
#define FRONT_STEERING 0

class MazeEscaper {
    private:
        ros::Publisher publisher;
        ros::Subscriber subscriber;
        ros::Subscriber subscriber_US;
        ros::Subscriber subscriber_IMU;
        ros::Subscriber subscriber_IMG;


        float min_length;
        float max_length;
        int32_t min_index;
        int32_t max_index;
        int32_t detect = 0;

        int32_t Leftsonic;
        int32_t Rightsonic;
        int32_t Backsonic_left;
        int32_t Backsonic_center;
        int32_t Backsonic_right;
        int32_t back_min;
        int32_t side_min;

        cv::Mat Frame;
        cv::Point center;
   
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
        void imageCallback(const sensor_msgs::Image& message);
        void UltraCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);

        float current_roll;
        float current_pitch;
        float current_yaw;
        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

        void drive(const int32_t steeringAngle);
        void back(const int32_t steeringAngle);
        void stop();

    public:
        MazeEscaper();
        void run();
};