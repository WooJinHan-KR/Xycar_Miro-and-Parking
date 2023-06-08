// go.hpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int32MultiArray.h>
#include <xycar_msgs/xycar_motor.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/Image.h>
#include "opencv2/opencv.hpp"

class GoNode {
    private:
        ros::Publisher publisher;
        ros::Subscriber subscriber;
        ros::Subscriber subscriber_US;
        ros::Subscriber subscriber_IMU;
        ros::Subscriber subscriber_IMG;


        float min_length;
        float max_length;
        int min_index;
        int max_index;
        int detect = 0;


        int Leftsonic;
        int Rightsonic;
        int  Backsonic_left;
        int  Backsonic_center;
        int  Backsonic_right;
        int back_min;
        int side_min;

        cv::Mat Frame;
        cv::Point center;

       
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
        void imageCallback(const sensor_msgs::Image& message);
        void UltraCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);


        ////////////////////////////////////////////////////////////////

        float current_roll;
        float current_pitch;
        float current_yaw;
        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

////////////////////////////////////////////////////////////////////////////
        void drive(int steeringAngle);
        void back(int steeringAngle);
        void stop();

    public:
        GoNode();
        void run();
};