// go.cpp
#include <go.hpp>
#include <algorithm>
#include <vector>
#include <iostream>

#define MAX_ULTRA_RANGE 140
int flag = 1;
int a =0;
GoNode::GoNode()
{
    ros::NodeHandle nh;
    this->subscriber = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &GoNode::scanCallback, this);
    this->publisher = nh.advertise<xycar_msgs::xycar_motor>("/xycar_motor", 1);

    this->subscriber_US = nh.subscribe<std_msgs::Int32MultiArray>("/xycar_ultrasonic", 1, &GoNode::UltraCallback, this);

    this->subscriber_IMG = nh.subscribe("/usb_cam/image_raw/", 1, &GoNode::imageCallback, this);

    this->subscriber_IMU = nh.subscribe<sensor_msgs::Imu>("/imu", 10, &GoNode::imuCallback, this);
}

void GoNode::imageCallback(const sensor_msgs::Image& message)
{
    cv::Mat src = cv::Mat(message.height, message.width, CV_8UC3, const_cast<uint8_t*>(&message.data[0]), message.step);
    cv::cvtColor(src, Frame, cv::COLOR_RGB2BGR);

    int first_count = 0;
    int second_count = 0;
    
    // Convert color to HSV
    cv::Mat hsv;
    cv::cvtColor(Frame, hsv, cv::COLOR_BGR2HSV);
    // Define range for blue color in HSV space
    cv::Scalar lower_blue = cv::Scalar(100, 200, 100);
    cv::Scalar upper_blue = cv::Scalar(140, 255, 255);
    // Threshold the HSV image to get only blue colors
    cv::Mat mask;
    cv::inRange(hsv, lower_blue, upper_blue, mask);
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& contour : contours) {
        // Get bounding rectangle for each contour
        cv::Rect rect = cv::boundingRect(contour);
        // Draw the rectangle
        cv::rectangle(Frame, rect, cv::Scalar(0, 255, 0));
        // Calculate and print the center point of the rectangle
        cv::Point center(rect.x + rect.width / 2, rect.y + rect.height / 2);
        //std::cout << "Center: " << center << std::endl;
        if(center.y > 180 && center.y < 300 && center.x > 320 && center.x < 520){
            first_count++;    
            first_count = first_count +a;          
        }

        if(first_count == 7)
        {
            flag = 4;
            a = 100;
        }
              
              

        //ROS_INFO_STREAM("center : " << center);
    }
    //cv::imshow("hi", Frame);
    //cv::waitKey(1);
}

void GoNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) 
{
    double roll, pitch, yaw;

    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );
 
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    this->current_roll = static_cast<float>(roll);
    this->current_pitch = static_cast<float>(pitch);
    this->current_yaw = static_cast<float>(yaw);
}

void GoNode::UltraCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{

    std::vector<int> UltraSonic(msg->data.begin(), msg->data.end());

    int Leftsonic;
    int Rightsonic;
    int Backsonic_left;
    int Backsonic_center;
    int Backsonic_right;

    Leftsonic = UltraSonic[0];
    Rightsonic = UltraSonic[4];
    Backsonic_left = UltraSonic[5];
    Backsonic_center = UltraSonic[6];
    Backsonic_right = UltraSonic[7];

    int back_min = MAX_ULTRA_RANGE;
    int side_min = MAX_ULTRA_RANGE;

    for(int i=5; i<8; i++){
        if(msg->data[i] < back_min)
            back_min = msg->data[i];
    }

    if(Leftsonic < Rightsonic)
        side_min = Leftsonic;
    else 
        side_min = Rightsonic;

    this->back_min = back_min;
    this->side_min = side_min;

    this->Leftsonic= Leftsonic;
    this->Rightsonic= Rightsonic;

    this->Backsonic_left= Backsonic_left;
    this->Backsonic_center= Backsonic_center;
    this->Backsonic_right= Backsonic_right;

    //ROS_INFO_STREAM("Leftsonic : " << Leftsonic);
    //ROS_INFO_STREAM("Rightsonic : " << Rightsonic);
}


void GoNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::vector<float> subset_ranges(msg->ranges.begin(), msg->ranges.end());

    float min_value = 100.0;
    float max_value = 0.0;
    int l_range = subset_ranges.size()/4;
    int r_range = subset_ranges.size()-l_range;
    int min_index = 0;
    int max_index = 0;
    

    for (int i = 1; i < l_range; ++i)
    {
        if (subset_ranges[i] < min_value &&subset_ranges[i] != 0)
        {
            min_value = subset_ranges[i];
            min_index = i;
        }

        if (subset_ranges[i] > max_value)
        {
            max_value = subset_ranges[i];
            max_index = i;
        }

    }

    for(int i = r_range; i < subset_ranges.size(); ++i)
    {
        if (subset_ranges[i] < min_value &&subset_ranges[i] != 0)
        {
            min_value = subset_ranges[i];
            min_index = i;
        }

        if (subset_ranges[i] > max_value)
        {
            max_value = subset_ranges[i];
            max_index = i;
        }

    }
      
    this->max_index = max_index;
    this->min_index = min_index;
    this->min_length = min_value;
    this->max_length = max_value;
}

void GoNode::drive(int max_index)
{
    xycar_msgs::xycar_motor motorMessage;
    float mXycarSpeed = 5.0f;
    motorMessage.angle = std::round(max_index);
    motorMessage.speed = std::round(mXycarSpeed);

    publisher.publish(motorMessage);
}

void GoNode::back(int max_index)
{
    xycar_msgs::xycar_motor motorMessage;
    float mXycarSpeed = -5.0f;
    motorMessage.angle = -max_index;
    motorMessage.speed = std::round(mXycarSpeed);

    publisher.publish(motorMessage);
}

void GoNode::stop()
{
    xycar_msgs::xycar_motor motorMessage;
    float mXycarSpeed = 0.0f;
    motorMessage.angle = 0;
    motorMessage.speed = std::round(mXycarSpeed);

    publisher.publish(motorMessage);
}

/*
void GoNode::Parking()
{
    xycar_msgs::xycar_motor motorMessage;

    

}*/

static constexpr int32_t kXycarSteeringAangleLimit = 50; ///< Xycar Steering Angle Limit
static constexpr double kFrameRate = 33.0;  

void GoNode::run()
{
    ros::Rate rate(kFrameRate);
    xycar_msgs::xycar_motor motorMessage;
    while (ros::ok())
    {
        ros::spinOnce();
        int angle = this->max_index;

        if(angle<=126 && angle>=1)
            angle *= -1;
        else if(angle >= 378 && angle <= 504)
            angle = 504 - angle;

        //parking back
        if(flag == 4)
        {
            drive(30);
            ROS_INFO_STREAM("Flag : " << flag);
            //drive((center.x-320)/2);
            if(min_length < 0.25){
            flag = 5;
            stop();
            ros::Duration duration (3.0);
            duration.sleep();
           }
        }

        if(flag == 5 && min_length < 0.25){
            back(25);
            flag = 5;
            ROS_INFO_STREAM("Flag : " << flag);
        }
        else if(min_length < 0.25 || flag == 2){   
            back(angle);
            flag = 2;
        }
        else if(flag==5 && back_min > 15){
            back(25);

        }
        if(back_min < 15 && flag == 5){
            flag = 1;
            a=0;
        }

        if((min_length > 0.55 || back_min < 40 )&& flag == 2) /*|| side_min <15 || back_min < 40 */
            flag = 1;
        
        //flag = 1 normal
        //flag = 0 avoid
        //flag = 2 back
        if(min_length <= 0.4 && flag == 1)
        {
            if(min_index <= 126)
                drive(50);
            else if(min_index >378 && min_index < 504)
                drive(-50);           
            flag = 0;
        }
        else if(min_length <= 0.5 && flag == 0)
        {
            if(min_index <= 126)
                drive(50);
            else if(min_index >378 && min_index < 504)
                drive(-50);
        }
        else if(min_length > 0.5 && flag == 0)
        {
            flag = 1;
            drive(angle);
        }
        else if(flag == 1 && min_length > 0.4)
            drive(angle);
  
        rate.sleep();
    }
}