#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <opencv_test/PoseStamped_Euler.h>

//Constant
#define PI 3.14159265

//Ultility functions
double radToDeg(double rad);

int main(int argc, char** argv){
    //Initialize ROS node
    ros::init(argc, argv, "pose_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle nh_param("~");

    //TF listener
    tf::TransformListener listener;

    //Parameters
    std::string frame_parent, frame_child;
    int frequency_publish;
    bool using_unit_degree;
    ros::Publisher pub_quaternion;
    ros::Publisher pub_euler;

    nh_param.param<std::string>("frame_parent", frame_parent, "/world");
    nh_param.param<std::string>("frame_child", frame_child, "/chassis");
    nh_param.param<int>("frequency_publish", frequency_publish, 10);
    nh_param.param<bool>("using_unit_degree", using_unit_degree, false);

    //Create publisher
    pub_quaternion = nh.advertise<geometry_msgs::PoseStamped>("pose_chassis/quaternion", 1);
    pub_euler = nh.advertise<opencv_test::PoseStamped_Euler>("pose_chassis/euler", 1);
    listener.waitForTransform(frame_parent, frame_child, ros::Time(), ros::Duration(1.0));
    ros::Rate rate(frequency_publish);

    while (nh.ok()){
        tf::StampedTransform transform;
        try{
            //Quaternion
            listener.lookupTransform(frame_parent, frame_child, ros::Time(0), transform);

            geometry_msgs::PoseStamped poseStamped_quaternion;
            poseStamped_quaternion.header.frame_id = frame_parent;
            poseStamped_quaternion.header.stamp = ros::Time::now();

            poseStamped_quaternion.pose.orientation.x = transform.getRotation().getX();
            poseStamped_quaternion.pose.orientation.y = transform.getRotation().getY();
            poseStamped_quaternion.pose.orientation.z = transform.getRotation().getZ();
            poseStamped_quaternion.pose.orientation.w = transform.getRotation().getW();

            poseStamped_quaternion.pose.position.x = transform.getOrigin().getX();
            poseStamped_quaternion.pose.position.y = transform.getOrigin().getY();
            poseStamped_quaternion.pose.position.z = transform.getOrigin().getZ();

            pub_quaternion.publish(poseStamped_quaternion);

            //Eulerian (Roll, Pitch, Yaw)
            tf::Quaternion quat = transform.getRotation();
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            opencv_test::PoseStamped_Euler poseStamped_euler;
            poseStamped_euler.header.frame_id = frame_parent;
            poseStamped_euler.header.stamp = ros::Time::now();

            poseStamped_euler.origin.x = transform.getOrigin().getX();
            poseStamped_euler.origin.y = transform.getOrigin().getY();
            poseStamped_euler.origin.z = transform.getOrigin().getZ();

            if (using_unit_degree){
                roll = radToDeg(roll);
                pitch = radToDeg(pitch);
                yaw = radToDeg(yaw);
            }
            poseStamped_euler.rotation.x = roll;
            poseStamped_euler.rotation.y = pitch;
            poseStamped_euler.rotation.z = yaw;
            pub_euler.publish(poseStamped_euler);
        } catch (tf::TransformException &ex){
            //Do nothing
        }
        rate.sleep();
    }
    return 1;
}

double radToDeg(double rad){
    return rad/PI*180.0;
}
