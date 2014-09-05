#include "ros/ros.h" 
#include "nav_msgs/Odometry.h" 
#include "geometry_msgs/Twist.h" 
#include "sensor_msgs/LaserScan.h" 
#include <cmath> 
#include <tf/LinearMath/Quaternion.h> 
#include <tf/LinearMath/Matrix3x3.h> 



class Base { 
public: 
    double x; 
    double y; 
    double heading;// in radians 

    Base() : x(0), y(0), heading(0) 
    { 
    } 
    void baseCallback(const nav_msgs::Odometry::ConstPtr& msg) { 
        double roll, pitch; 
        x = (-1)* msg->pose.pose.position.y; 
        y = msg->pose.pose.position.x; 
        tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x, \
                                          msg->pose.pose.orientation.y, \
                                          msg->pose.pose.orientation.z, \
                                          msg->pose.pose.orientation.w); 
        tf::Matrix3x3(q).getRPY(roll, pitch, heading); 
    } 
}; 

class Pose { 
public: 
    double x; 
    double y; 
    double heading;// in radians 

    Pose() : x(0), y(0), heading(0) 
    { 
    } 
    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) { 
        double roll, pitch; 
        x = msg->pose.pose.position.x; 
        y = msg->pose.pose.position.y; 

        tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x, \
                                          msg->pose.pose.orientation.y, \
                                          msg->pose.pose.orientation.z, \
                                          msg->pose.pose.orientation.w); 
        tf::Matrix3x3(q).getRPY(roll, pitch, heading); 
    } 
}; 


class StageBot { 
public: 
    Base base; 
    Pose pose; 

    StageBot(ros::NodeHandle& nh, int robotID): ID(robotID) 
    { 
        commandPub = nh.advertise<geometry_msgs::Twist>("/robot_" + \
                                                        boost::lexical_cast<std::string>(robotID) + \
                                                        "/cmd_vel", 1); 
        laserSub = nh.subscribe("/robot_" +\
                                boost::lexical_cast<std::string>(robotID) +\
                                "/base_scan", 1, \
                                &StageBot::laserCallback, this); 

        baseSub = nh.subscribe("/robot_" + \
                               boost::lexical_cast<std::string>(robotID) + \
                               "/base_pose_ground_truth", 1, \
                               &Base::baseCallback, &base ); 

        poseSub = nh.subscribe("/robot_" + \
                               boost::lexical_cast<std::string>(robotID) + \
                               "/odom", 1, \
                               &Pose::poseCallback, &pose ); 
    } 

    void move(double linVelx, double angVelz) 
    { 
        geometry_msgs::Twist msg; 

        msg.linear.x = linVelx; 
        msg.angular.z = angVelz; 
        commandPub.publish(msg); 
    } 

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) 
    { 
        //TODO 
    } 

protected: 
    ros::Publisher commandPub; 
    ros::Subscriber laserSub; 
    ros::Subscriber baseSub; 
    ros::Subscriber poseSub; 
    int ID; 
}; 

class Controller { 
public: 
    Controller(){} 

    double getAngularDistance(StageBot& robot, StageBot& goal) 
    { 
        return atan2(goal.pose.y - robot.pose.y, \
                     goal.pose.x - robot.pose.x) \
                   -robot.pose.heading; 
    } 

    double getLinearDistance(StageBot& robot, StageBot& goal) 
    { 
        return sqrt(pow(goal.pose.x - robot.pose.x, 2) \
                    + pow(goal.pose.y - robot.pose.y, 2)); 
    } 
}; 

int main(int argc, char **argv) 
{ 
    ros::init(argc, argv, "controller"); 
    ros::NodeHandle n("controller"); 
    StageBot robot(n, 0); 
    StageBot goal(n, 1); 
    Controller controller; 

    double distance, angVelz; 

    ros::Rate rate(20); 

    while (ros::ok()) { 

        angVelz = controller.getAngularDistance(robot,goal); 

        distance = controller.getLinearDistance(robot,goal); 

        if (std::abs(angVelz) > 1e-4){ 
            robot.move(0, 3 * angVelz); 

        } 
        else { 
            robot.move(distance * 0.8 ,0.0); 
        } 


        ros::spinOnce(); 
        rate.sleep(); 
    } 
    return EXIT_SUCCESS; 
}

