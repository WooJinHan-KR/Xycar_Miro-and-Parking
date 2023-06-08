// go.cpp
#include <go.hpp>
#include <algorithm>
#include <vector>
#include <iostream>

#define MAX_ULTRA_RANGE 140
int flag = 1;

GoNode::GoNode()
{
    ros::NodeHandle nh;
    this->subscriber = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &GoNode::scanCallback, this);
    this->publisher = nh.advertise<xycar_msgs::xycar_motor>("/xycar_motor", 1);
    
    this->subscriber_IMU = nh.subscribe<sensor_msgs::Imu>("/imu", 10, &GoNode::imuCallback, this);

    this->subscriber_US = nh.subscribe<std_msgs::Int32MultiArray>("/xycar_ultrasonic", 1, &GoNode::UltraCallback, this);


}

void GoNode::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    this->current_roll = static_cast<float>(roll);
    this->current_pitch = static_cast<float>(pitch);
    this->current_yaw = static_cast<float>(yaw);


    //ROS_INFO_STREAM("i : " << "  "<< this->current_yaw);


}

void GoNode::UltraCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
{

    std::vector<int> UltraSonic(msg->data.begin(), msg->data.end());

    int Leftsonic;
    int  Rightsonic;
    int  Backsonic_left;
    int Backsonic_center;
    int Backsonic_right;

 
    Leftsonic = UltraSonic[0];
    Rightsonic = UltraSonic[4];

    Backsonic_left = UltraSonic[5];
    Backsonic_center = UltraSonic[6];
    Backsonic_right = UltraSonic[7];


    
    this->Leftsonic= Leftsonic;
    this->Rightsonic= Rightsonic;

    this->Backsonic_left= Backsonic_left;
    this->Backsonic_center= Backsonic_center;
    this->Backsonic_right= Backsonic_right;

    ROS_INFO_STREAM("Leftsonic : " << Leftsonic);
    ROS_INFO_STREAM("Rightsonic : " << Rightsonic);
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
    bool action_running = false;
    std::vector<float> arr(20, 0.0);

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

        //ROS_INFO_STREAM("i : " << i << "  "<< subset_ranges[i]);
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

        //ROS_INFO_STREAM("i : " << i << "  "<< subset_ranges[i]);
    }


    /*for(int i=0; i<l_range; i++){
        arr[i/14] += subset_ranges[i];
    }
    for(int i=r_range; i<subset_ranges.size(); ++i){
        arr[i/14 - r_range/14] += subset_ranges[i];
    }

    float a = 0.0;
    int index = 0;

    for(int i=0; i<arr.size(); i++)
    {  
        if(arr[i] > a){
            a = arr[i];
            index = i;
        }
    }

    if(index < 3)
        index = 0;
    else if(index < 6)
        index = -25;
    else if(index <= 8)
        index = -50;
    else if(index < 12)
        index = 50;
    else if(index < 15)
        index = 25;
    else if(index < 18)
        index = 0;*/
      
    this->max_index = max_index;
    this->min_index = min_index;
    this->min_length = min_value;
    this->max_length = max_value;
}

void GoNode::drive(int max_index)
{
    xycar_msgs::xycar_motor motorMessage;
    float mXycarSpeed = 5.0;
    motorMessage.angle = std::round(max_index);
    motorMessage.speed = std::round(mXycarSpeed);

    publisher.publish(motorMessage);
}

void GoNode::back()
{
    xycar_msgs::xycar_motor motorMessage;
    float mXycarSpeed = -5.0;
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
        //ROS_INFO_STREAM("max_index : " << this->max_index);
        //ROS_INFO_STREAM("min_index : " << this->min_index);
        //ROS_INFO_STREAM("max_value : " << this->max_length);
        //ROS_INFO_STREAM("min_value : " << this->min_length);
        ros::spinOnce();
        int angle = this->max_index;

        /////////////////////////////////////////////////////

        /*ROS_INFO_STREAM("left : " << this->Leftsonic);
        ROS_INFO_STREAM("right : " << this->Rightsonic);

        ROS_INFO_STREAM("Backsonic_left : " << this->Backsonic_left);
        ROS_INFO_STREAM("Backsonic_center : " << this->Backsonic_center);
        ROS_INFO_STREAM("Backsonic_right : " << this->Backsonic_right);*/
        

        ///////////////////////////////////////////////

        



        
        if(angle<=126 && angle>=1)
            angle *= -1;
        else if(angle >= 378 && angle <= 504)
            angle = 504 - angle;

        /*
        //parking back
        if(Leftsonic == MAX_ULTRA_RANGE || Rightsonic == MAX_ULTRA_RANGE)
        {
            back();
            flag = 2;
        }
        
        else if(Backsonic_left<20 || Backsonic_center<20 || Backsonic_right<20)
        {
            flag = 3;
        }
        */
        

        //back
        if(min_length <= 0.15)
        {   
            back();
            flag = 2;
        }
        else if(min_length > 0.4)
        {
            flag = 1;
        }


        //flag = 1 normal
        if(min_length <= 0.3 && flag == 1)
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
        else if(flag == 1 && min_length > 0.3)
        {
            drive(angle);
        }
   
        //ROS_INFO_STREAM("angle : " << angle);
        /*if(flag == 1)
        {
            ros::Time a_little(0.001);
            ros::Duration two_sec(2.0);
        }
        else
        {
            rate.sleep();
        }*/

        rate.sleep();
    }
}