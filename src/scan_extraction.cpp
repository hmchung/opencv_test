/*////////////////////////////////////////////////////////////////////////////////////////////////////
Author:         Minh-Chung HOANG
Email:          MINHCHUN001@e.ntu.edu.sg
Last modified:  Tue 8th Jan 16
Purposes:
- Subsribe to raw laser scan, topic /scan, type sensor_msgs/LaserScan
- Extract and devide the range readings into multiple segments
- The number of segments and amount of data is specified by pre-defined rations
    - param "/scan/rations/count"
    - param "/scan/rations/0"
    - param "/scan/rations/1"
    - param "/scan/rations/2" ...
    - param "/scan/rations/n"
- The number of segments and assigned rations must be consistent
- Publish data to
    - /scan/data/segment_0
    - /scan/data/segment_1
    - /scan/data/segment_2 ...
    - /scan/data/segment_n
////////////////////////////////////////////////////////////////////////////////////////////////////*/

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>

//Standard C libs
#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <string>

using namespace std;

//Function declarations
void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan2);
std::string int_to_string(int myInt);
//Parameters
std::string nodeName = "scan_extractor";
int segmentCount = 0;
int tempRation = 0;
ros::Publisher* pubPtr;
int* rationPtr;
int totalRation = 0;

int main(int argc, char** argv){
    ros::init(argc, argv, nodeName);
    ros::NodeHandle nh;
    ros::NodeHandle nh_param("~");
    nh_param.param<int>("/scan/rations/count", segmentCount, 0);

    if (segmentCount > 0){
        ros::Subscriber scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, scanCallBack);
        int rations[segmentCount];
        ros::Publisher pubs[segmentCount];
        totalRation = 0;
        for (int i = 0; i < segmentCount; i++){
            std::string param_name = "/scan/rations/" + int_to_string(i);
            nh_param.param<int>(param_name, tempRation, 0);
            totalRation += tempRation;
            rations[i] = tempRation;
            pubs[i] = nh.advertise<sensor_msgs::LaserScan>("/scan/data/segment_" + int_to_string(i), 1);
            ROS_INFO("ration %d: %d", i, tempRation);
        }
        rationPtr = rations;
        pubPtr = pubs;

        ros::spin();
    }
    return 0;
}

void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan2){
    //Get the number of original laser reading
    int originalCount = scan2->ranges.size();

    //Get the startIndex and readingCount
    int startIndices[segmentCount];
    int readingCounts[segmentCount];
    int tempIndex = 0;
    for (int i = 0; i < segmentCount; i++){
        startIndices[i] = tempIndex;
        readingCounts[i] = originalCount / totalRation * rationPtr[i];
        tempIndex += readingCounts[i];
    }

    sensor_msgs::LaserScan scan;
    for (int i = 0; i < segmentCount; i++){
        //Data sets range params
        int readingCount = readingCounts[i];
        int startIndex = startIndices[i];

        //populate the LaserScan message
        sensor_msgs::LaserScan scan;
        scan.header.stamp = scan2->header.stamp;
        scan.header.frame_id = scan2->header.frame_id;
        scan.angle_min = scan2->angle_min;
        scan.angle_max = scan2->angle_max;
        scan.angle_increment = scan2->angle_increment;
        scan.time_increment = scan2->time_increment;
        scan.range_min = 0.0;
        scan.range_max = 100.0;
        scan.ranges.resize(readingCount);
        for(int j = 0; j < readingCount; ++j){
            scan.ranges[j] = scan2->ranges[startIndex + j];
        }
        pubPtr[i].publish(scan);
    }
}

std::string int_to_string(int myInt){
    int tempInt = myInt;
    char tempChar;
    std::string myStr;
    if (myInt == 0){
        myStr += '0';
    } else {
        while (tempInt != 0){
            tempChar = tempInt%10 + '0';
            tempInt = (tempInt - tempInt%10) / 10;
            myStr = tempChar + myStr;
        }
        if (myInt < 0){
            myStr = "-" + myStr;
        }
    }
    return myStr;
}
