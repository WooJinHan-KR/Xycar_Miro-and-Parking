// MazeEscape.cpp
#include <algorithm>
#include <MazeEscape.hpp>
#include <iostream>
#include <vector>

int32_t Mode = NORMAL;
int32_t Parking_Token = 0;
bool First_Parking = false;

MazeEscaper::MazeEscaper()
{
    ros::NodeHandle nh;
    this->subscriber = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &MazeEscaper::scanCallback, this);
    this->publisher = nh.advertise<xycar_msgs::xycar_motor>("/xycar_motor", 1);

    this->subscriber_US = nh.subscribe<std_msgs::int32_t32MultiArray>("/xycar_ultrasonic", 1, &MazeEscaper::UltraCallback, this);
    this->subscriber_IMG = nh.subscribe("/usb_cam/image_raw/", 1, &MazeEscaper::imageCallback, this);
    this->subscriber_IMU = nh.subscribe<sensor_msgs::Imu>("/imu", 10, &MazeEscaper::imuCallback, this);
}

void MazeEscaper::imageCallback(const sensor_msgs::Image& message)
{
    cv::Mat src = cv::Mat(message.height, message.width, CV_8UC3, const_cast<uint32_t8_t*>(&message.data[0]), message.step);
    cv::cvtColor(src, Frame, cv::COLOR_RGB2BGR);

    int32_t Parking_Token = 0;

    cv::Mat hsv;

    cv::cvtColor(Frame, hsv, cv::COLOR_BGR2HSV);
    cv::Scalar lower_blue = cv::Scalar(100, 200, 100);
    cv::Scalar upper_blue = cv::Scalar(140, 255, 255);

    cv::Mat mask;
    cv::inRange(hsv, lower_blue, upper_blue, mask);
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    
    for (const auto& contour : contours) {
        cv::Rect rect = cv::boundingRect(contour);
        cv::rectangle(Frame, rect, cv::Scalar(0, 255, 0));
        cv::Point center(rect.x + rect.width / 2, rect.y + rect.height / 2);
        
        if(center.y > 180 && center.y < 300 && center.x > 170 && center.x < 520){
            ++first_count;    
            first_count += Parking_Token;          
        }

        if(first_count == 7)
        {
            Mode = PARKING;
            Parking_Token = STANDBY;
        }           
    }
}

void MazeEscaper::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) 
{
    float roll, pitch, yaw;

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

void MazeEscaper::UltraCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{

    std::vector<int32_t> UltraSonic(msg->data.begin(), msg->data.end());

    int32_t Leftsonic_;
    int32_t Rightsonic_;
    int32_t Backsonic_left_;
    int32_t Backsonic_center_;
    int32_t Backsonic_right_;

    Leftsonic_ = UltraSonic[0];
    Rightsonic_ = UltraSonic[4];
    Backsonic_left_ = UltraSonic[5];
    Backsonic_center_ = UltraSonic[6];
    Backsonic_right_ = UltraSonic[7];

    int32_t back_min_ = MAX_ULTRA_RANGE;
    int32_t side_min_ = MAX_ULTRA_RANGE;

    for(int32_t i=5; i<8; ++i)
        if(msg->data[i] < back_min)
            back_min_ = msg->data[i];

    if(Leftsonic_ < Rightsonic_)
        side_min_ = Leftsonic_;
    else 
        side_min_ = Rightsonic_;

    this->back_min = back_min_;
    this->side_min = side_min_;

    this->Leftsonic= Leftsonic_;
    this->Rightsonic= Rightsonic_;

    this->Backsonic_left= Backsonic_left_;
    this->Backsonic_center= Backsonic_center_;
    this->Backsonic_right= Backsonic_right_;
}

void MazeEscaper::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::vector<float> subset_ranges(msg->ranges.begin(), msg->ranges.end());

    float min_value_ = 100.0f;
    float max_value_ = 0.0f;
    int32_t min_index_ = 0;
    int32_t max_index_ = 0;
    
    for (int32_t i = LEFT_END; i < LEFT_START; ++i){// 1 <= i < 126
        if (subset_ranges[i] < min_value_ &&subset_ranges[i] != 0){
            min_value_ = subset_ranges[i];
            min_index_ = i;
        }

        if (subset_ranges[i] > max_value_){
            max_value_ = subset_ranges[i];
            max_index_ = i;
        }
    }

    for(int j = RIGHT_START; j < RIGHT_END; ++j){// 378 <= j < 504
        if (subset_ranges[j] < min_value_ && subset_ranges[j] != 0){
            min_value_ = subset_ranges[j];
            min_index_ = j;
        }

        if (subset_ranges[j] > max_value_){
            max_value_ = subset_ranges[j];
            max_index_ = j;
        }
    }
      
    this->max_index = max_index_;
    this->min_index = min_index_;
    this->min_length = min_value_;
    this->max_length = max_value_;
}

void MazeEscaper::drive(const int32_t max_index)
{
    xycar_msgs::xycar_motor motorMessage;
    float mXycarSpeed = 9.0f;
    motorMessage.angle = std::round(max_index);
    motorMessage.speed = std::round(mXycarSpeed);

    publisher.publish(motorMessage);
}

void MazeEscaper::back(const int32_t max_index)
{
    xycar_msgs::xycar_motor motorMessage;
    float mXycarSpeed = -5.0f;
    motorMessage.angle = -max_index;
    motorMessage.speed = std::round(mXycarSpeed);

    publisher.publish(motorMessage);
}

void MazeEscaper::stop()
{
    xycar_msgs::xycar_motor motorMessage;
    float mXycarSpeed = 0.0f;
    motorMessage.angle = 0;
    motorMessage.speed = std::round(mXycarSpeed);

    publisher.publish(motorMessage);
}

void MazeEscaper::run()
{
    static constexpr double kFrameRate = 33.0;  
    ros::Rate rate(kFrameRate);
    xycar_msgs::xycar_motor motorMessage;

    while (ros::ok())
    {
        ros::spinOnce();
        int32_t angle = this->max_index;

        if(angle <= LEFT_START && angle >= LEFT_END)
            angle *= -1;
        else if(angle >= RIGHT_START && angle <= RIGHT_END)
            angle = RIGHT_END - angle;

        switch (Mode){

        case PARKING :
            if(min_length < 0.35 && First_Parking == false)
                drive(FRONT_STEERING - 10);
            else if(min_length > 0.35 && First_Parking == false)
                drive(FULL_RIGHT_STEERING - 15);
            else if(min_length > 0.35 && First_Parking == true)
                drive(FULL_LEFT_STEERING + 25);
            else if(min_length < 0.35 && First_Parking == true)
                drive(FRONT_STEERING + 5);

            if(min_length < 0.25 && (First_Parking == true || First_Parking == false)){
                Mode = OUT;
                stop();
                if(First_Parking == true)
                    break;
                ros::Duration duration (3.0);
                duration.sleep();
           }
            break;
        
        case OUT :    
            back(FRONT_STEERING + 12);
            if(back_min < 15){
                Mode = NORMAL;
                Parking_Token = 0;
                First_Parking = true;
            }
            break;
        
        case BACK :
            back(angle);

            if(min_length > 0.55 || back_min < 35)
                Mode = NORMAL;
            break;
        
        case NORMAL :
            drive(angle);

            if(min_length < 0.4){
                if(min_index <= LEFT_START)
                    drive(FULL_RIGHT_STEERING);
                else if(min_index >RIGHT_START && min_index < RIGHT_END)
                    drive(FULL_LEFT_STEERING); 

                Mode = AVOID;
            }

            if(min_length < 0.25){
                back(angle);
                Mode = BACK;
            }
            break;

        case AVOID :
            if(min_index <= LEFT_START)
               drive(FULL_RIGHT_STEERING);
            else if(min_index > RIGHT_START && min_index < RIGHT_END)
                drive(FULL_LEFT_STEERING);

            if(min_length > 0.5){
                Mode = NORMAL;
                drive(angle);
            }

            if(min_length < 0.25){
                back(angle);
                Mode = BACK;
            }
            break;
        
        default:
            break;
        }
  
        rate.sleep();
    }
}