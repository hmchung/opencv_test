/*//////////////////////////////////////////////////
Author:         Minh-Chung HOANG
Email:          MINHCHUN001@e.ntu.edu.sg
Last modified:  Tue 5th Jan 16
Purposes:
- Listen to IMU data <sensor_msgs::imu>
- Publish rotation accordingly in deg and rad
- The rate of publishing depends on rate of imu data
//////////////////////////////////////////////////*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>
//Constant
#define PI 3.14159265

//Ultility functions
double radToDeg(double rad);
void callback_imu(const sensor_msgs::Imu::ConstPtr& imuMsg);

//Parameters
std::string topic_input_imu, topic_output;
std::string frame_imu;

geometry_msgs::Quaternion quat;

int main(int argc, char** argv){
    //Initialize ROS node
    ros::init(argc, argv, "imu_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle nh_param("~");

//    pub_quaternion = new ros::Publisher();
//    pub_euler = new ros::Publisher();
//    imu_listener = new ros::Subscriber();

    nh_param.param<std::string>("topic_output", topic_output, "/imu_output");
    nh_param.param<std::string>("topic_input_imu", topic_input_imu, "/mavros/imu/data");
    nh_param.param<std::string>("frame_imu", frame_imu, "/imu");

    //ROS entities
    ros::Publisher pub_quaternion = nh.advertise<geometry_msgs::QuaternionStamped>(topic_output + "/rotation/quaternion", 1);
    ros::Publisher pub_euler_deg = nh.advertise<geometry_msgs::Vector3Stamped>(topic_output + "/rotation/euler/deg", 1);
    ros::Publisher pub_euler_rad = nh.advertise<geometry_msgs::Vector3Stamped>(topic_output + "/rotation/euler/rad", 1);
    ros::Subscriber imu_listener = nh.subscribe<sensor_msgs::Imu>(topic_input_imu, 1000, callback_imu);

    //Triggering callbacks
    while (nh.ok()){
        //Publishing rotation in Eulerian angles
            //Using TF::Quaternion - Stupid, but works

        double roll, pitch, yaw;
        tf::Quaternion quatTF = tf::Quaternion(quat.x, quat.y, quat.z, quat.w);
        tf::Matrix3x3(quatTF).getRPY(roll, pitch, yaw);

        geometry_msgs::Vector3Stamped vector_rad;
        vector_rad.header.frame_id = frame_imu;
        vector_rad.header.stamp = ros::Time::now();
        vector_rad.vector.x = roll;
        vector_rad.vector.y = pitch;
        vector_rad.vector.z = yaw;
        pub_euler_rad.publish(vector_rad);

        geometry_msgs::Vector3Stamped vector_deg;
        vector_deg.header.frame_id = frame_imu;
        vector_deg.header.stamp = ros::Time::now();
        vector_deg.vector.x = radToDeg(roll);
        vector_deg.vector.y = radToDeg(pitch);
        vector_deg.vector.z = radToDeg(yaw);
        pub_euler_deg.publish(vector_deg); //Degrees

        //Publishing rotation in quaternions
        geometry_msgs::QuaternionStamped quatStamped;
        quatStamped.header.frame_id = frame_imu;
        quatStamped.header.stamp = ros::Time::now();

        quatStamped.quaternion = quat;

        pub_quaternion.publish(quatStamped);
        ros::spinOnce();
    }

    return 1;
}

void callback_imu(const sensor_msgs::Imu::ConstPtr& imuMsg){
    quat = imuMsg->orientation;
}

double radToDeg(double rad){
    return rad/PI*180.0;
}
