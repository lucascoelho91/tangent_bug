#include "ros/ros.h" 
#include "nav_msgs/Odometry.h" 
#include "geometry_msgs/Twist.h" 
#include "sensor_msgs/LaserScan.h" 
#include <cmath> 
#include <tf/LinearMath/Quaternion.h> 
#include <tf/LinearMath/Matrix3x3.h> 


class pose_xy{
	public:
		double x;
		double y;
		double heading;


		static double getAngularDistance(pose_xy& p1, pose_xy& p2) 
	    { 
	        return atan2(p2.y - p1.y, p2.x - p1.x)  -p1.heading; 
	    } 

	    static double getLinearDistance(pose_xy& p1, pose_xy& p2)  
	    { 
	        return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2)); 
	    }

	    pose_xy(double xp = 0, double yp = 0, double hp = 0)
	    {
	    	x = xp;
	    	y = yp;
	    	heading = hp;
	    }
};

class Base { 
public: 
    pose_xy p; 

    Base(){}

    void baseCallback(const nav_msgs::Odometry::ConstPtr& msg) { 
        double roll, pitch; 
        p.x = (-1)* msg->pose.pose.position.y; 
        p.y = msg->pose.pose.position.x; 
        tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x, \
                                          msg->pose.pose.orientation.y, \
                                          msg->pose.pose.orientation.z, \
                                          msg->pose.pose.orientation.w); 
        tf::Matrix3x3(q).getRPY(roll, pitch, p.heading); 
    } 
}; 

class Pose { 
public: 
    pose_xy p;

    Pose() {} 
    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) { 
        double roll, pitch; 
        p.x = msg->pose.pose.position.x; 
        p.y = msg->pose.pose.position.y; 

        tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x, \
                                          msg->pose.pose.orientation.y, \
                                          msg->pose.pose.orientation.z, \
                                          msg->pose.pose.orientation.w); 
        tf::Matrix3x3(q).getRPY(roll, pitch, p.heading); 
    } 
}; 


class Laser{
	sensor_msgs::LaserScan laser;
public:
	Laser() {}

	void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) 
    { 
      	laser = msg*;
    } 

    pose_xy getMinimumDistanceRobotToPoint(pose_xy robot_pose)
    {
    	for(int i = 0, float minimum = laser.range_max, int min_index = 0; i < laser.ranges.size(); i++)
    	{
    		if (laser.ranges[i] > laser.ranges.min && laser.ranges[i] < minimum){
    			minimum = laser.ranges[i];
    			min_index = i;
    		}
    	}
    	pose_xy point;
    	double ang_point =robot_pose.p.heading - (laser.angle_increment * min_index + laser.angle_min);

    	point.x = robot_pose.p.x + minimum*cos(ang_point);
    	point.y = robot_pose.p.y + minimum*sen(ang_point);
    	point.heading = ang_point;

    	return point;
    }

    pose_xy getPointMinimumDistanceGoal(pose_xy goal)
    {
    	float laserDist;
    	double ang_point, dist;
    	double minDist;
    	pose_xy minPoint;
    	for(int i = 0; i < laser.ranges.size(); i++)
    	{
    		if (laser.ranges[i] > laser.range_min){
    			laserDist = laser.ranges[i];
    			
    			ang_point = goal.p.heading - (laser.angle_increment * i + laser.angle_min);

		    	point.x = robot_pose.p.x + laserDist*cos(ang_point);
		    	point.y = robot_pose.p.y + laserDist*sen(ang_point);
		    	point.heading = ang_point;

		    	dist = pose_xy::getLinearDistance(goal, point);
		    	if(dist < minDist)
		    		minPoint = point;
    		}
    	}
    	return minPoint;
    }

    	// checks if there is an obstacle in front of the robot
    	bool obstacle_in_path()
    	{
    		int midpoint = (laser.angle_max - laser.angle_min)/(laser.angle_increment*2);

    		if(laser.ranges[midpoint] > laser.range_min && laser.ranges[midpoint] < laser.range_max){
  				return true;
  			}
  			else{
  				return false;
  			}
    	}

    	pose_xy findClosestTangentPoint(pose_xy goal, pose_xy robot)
    	{

	    	float laserDist;
	    	double ang_point, dist;
	    	double minDist;
	    	pose_xy minPoint;
	    	bool onObstacle;

	    	for(int i = 0; i < laser.ranges.size(); i++)
	    	{
	    		if (laser.ranges[i] > laser.range_min){

	    			if( (laser.ranges[i] < laser.range_max && onObstacle == false) || (laser.ranges[i] >= laser.range_max && onObstacle == true))
	    			{
		    			laserDist = laser.ranges[i];
		    			
		    			ang_point = goal.p.heading - (laser.angle_increment * i + laser.angle_min);

				    	point.x = robot_pose.p.x + laserDist*cos(ang_point);
				    	point.y = robot_pose.p.y + laserDist*sen(ang_point);
				    	point.heading = ang_point;

				    	dist = pose_xy::getLinearDistance(goal, point) +  pose_xy::getLinearDistance(point, robot);
				    	if(dist < minDist)
				    		minPoint = point;
				    }
	    		}
	    	}
	    	return minPoint;
	    }

};

class StageBot { 
public: 
    Base base; 
    Pose pose; 
    Laser laser;

    StageBot(ros::NodeHandle& nh, int robotID): ID(robotID) 
    { 
        commandPub = nh.advertise<geometry_msgs::Twist>("/robot_" + \
                                                        boost::lexical_cast<std::string>(robotID) + \
                                                        "/cmd_vel", 1); 
        laserSub = nh.subscribe("/robot_" +\
                                boost::lexical_cast<std::string>(robotID) +\
                                "/base_scan", 1, \
                                &Laser::laserCallback, &laser); 

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

    

protected: 
    ros::Publisher commandPub; 
    ros::Subscriber laserSub; 
    ros::Subscriber baseSub; 
    ros::Subscriber poseSub; 
    int ID; 
}; 

class Controller { 
public: 

	pose_xy Oi;
	bool last_dir;
	double d_reach;
	double d_followed;


    Controller(){} 

    double getAngularDistance(StageBot& robot, StageBot& goal) 
    { 
        return atan2(goal.pose.p.y - robot.pose.p.y, \
                     goal.pose.p.x - robot.pose.p.x) \
                   -robot.pose.p.heading; 
    } 

    double getLinearDistance(StageBot& robot, StageBot& goal) 
    { 
        return sqrt(pow(goal.pose.p.x - robot.pose.p.x, 2) \
                    + pow(goal.pose.p.y - robot.pose.p.y, 2)); 
    }

    double getLinearDistance(pose_xy p1, pose_xy p2) 
    { 
        return pose_xy::getLinearDistance(p1, p2);
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
