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
      	laser = *msg;
    } 

    pose_xy getMinimumDistanceRobotToPoint(pose_xy robot_pose)
    {
    	float minimum = laser.range_max;
    	int min_index = 0;
    	for(int i = 0; i < laser.ranges.size(); i++)
    	{
    		if (laser.ranges[i] > laser.range_min && laser.ranges[i] < minimum){
    			minimum = laser.ranges[i];
    			min_index = i;
    		}
    	}
    	pose_xy point;
    	double ang_point =robot_pose.heading - (laser.angle_increment * min_index + laser.angle_min);

    	point.x = robot_pose.x + minimum*cos(ang_point);
    	point.y = robot_pose.y + minimum*sin(ang_point);
    	point.heading = ang_point;

    	return point;
    }

    pose_xy getPointMinimumDistanceGoal(pose_xy goal, pose_xy robot_pose)
    {
    	float laserDist;
    	double ang_point, dist;
    	double minDist;
    	pose_xy minPoint, point;
    	for(int i = 0; i < laser.ranges.size(); i++)
    	{
    		if (laser.ranges[i] > laser.range_min){
    			laserDist = laser.ranges[i];
    			
    			ang_point = goal.heading - (laser.angle_increment * i + laser.angle_min);

		    	point.x = robot_pose.x + laserDist*cos(ang_point);
		    	point.y = robot_pose.y + laserDist*sin(ang_point);
		    	point.heading = ang_point;

		    	dist = pose_xy::getLinearDistance(goal, point);
		    	if(dist < minDist)
		    		minPoint = point;
    		}
    	}
    	return minPoint;
    }

    	// checks if there is an obstacle in front of the robot
    	bool obstacle_in_path(pose_xy goal, pose_xy robot)
    	{
    		float phi = atan2(goal.y, goal.x);
    		float rho = robot.heading - phi;

    		float ang = -laser.angle_min - rho;

    		int index = ang / laser.angle_increment;
    		if(laser.ranges[index] > laser.range_min && laser.ranges[index] < laser.range_max){
  				return true;
  			}
  			else{
  				return false;
  			}
    	}

    	pose_xy find_dReach(pose_xy goal, pose_xy robot)
    	{
    		float phi = atan2(goal.y, goal.x);
    		float rho = robot.heading - phi;

    		float ang = -laser.angle_min - rho;

    		int index = ang / laser.angle_increment;
    		
    		float dist = laser.ranges[index];

    		pose_xy dReach;
    		dReach.x = dist*cos(phi) + robot.x;
    		dReach.y = dist*sin(phi) + robot.y;
    		dReach.heading = phi;
    		return dReach;
    	}
    	pose_xy findClosestTangentPoint(pose_xy goal, pose_xy robot_pose)
    	{

	    	float laserDist;
	    	double ang_point, dist;
	    	double minDist;
	    	pose_xy minPoint, point;
	    	bool onObstacle;

	    	for(int i = 0; i < laser.ranges.size(); i++)
	    	{
	    		if (laser.ranges[i] > laser.range_min){

	    			if( (laser.ranges[i] < laser.range_max && onObstacle == false) || (laser.ranges[i] >= laser.range_max && onObstacle == true))
	    			{
		    			laserDist = laser.ranges[i];
		    			
		    			ang_point = goal.heading - (laser.angle_increment * i + laser.angle_min);

				    	point.x = robot_pose.x + laserDist*cos(ang_point);
				    	point.y = robot_pose.y + laserDist*sin(ang_point);
				    	point.heading = ang_point;

				    	dist = pose_xy::getLinearDistance(goal, point) +  pose_xy::getLinearDistance(point, robot_pose);
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
   	double d; 
   	double kv;
   	double kw;

    StageBot(ros::NodeHandle& nh, int robotID, double dist=0.1, double kvel = 1, double kwp = 0.05): ID(robotID) 
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
        this->d = dist;
        this->kv = kvel;
        this->kw = kwp;
    } 

    void move(double xvel, double yvel) 
    { 
        geometry_msgs::Twist msg; 

        double v, w;

        v = kv*( cos(base.p.heading)*xvel + sin(base.p.heading)*yvel );
        w = kw*( -sin(base.p.heading)*yvel/d + cos(base.p.heading)*yvel/d );
                

        msg.linear.x = v; 
        msg.angular.z = w; 
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
	double last_dist;

    Controller(){
    	last_dist = 0;
    } 

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

    static pose_xy getRobotGoalVector(pose_xy p1, pose_xy p2)
    {
    	pose_xy s;
    	s.x = p1.x - p2.x;
    	s.y = p1.y - p2.y;
    	s.heading  = atan2(s.y, s.x);
    	return s;
    }

}; 

int main(int argc, char **argv) 
{ 
    ros::init(argc, argv, "controller"); 
    ros::NodeHandle n("controller"); 
    StageBot robot(n, 0, 1, 1, 0.05); 
    StageBot goal(n, 1); 
    Controller controller; 

    double K = 1; 
    pose_xy Oi, goalRobotV;
    double dist;
    double last_dist=0;

    double dReached, dFollow;
    pose_xy dFollowp;

    ros::Rate rate(20); 

    while (ros::ok()) { 
    	ros::spinOnce();
		rate.sleep();

    	if (pose_xy::getLinearDistance(goal.pose.p, robot.pose.p) < 0.1)
    	{
    		ROS_INFO("GOAL REACHED!");
        	robot.move(0, 0);

    		return EXIT_SUCCESS;
		}
		else
		{
	        if(robot.laser.obstacle_in_path(goal.pose.p, robot.pose.p))
	        {
	        	Oi = robot.laser.findClosestTangentPoint(goal.pose.p, robot.pose.p);
	        }

	        dist = pose_xy::getLinearDistance(goal.pose.p, Oi) + pose_xy::getLinearDistance(Oi, robot.pose.p);
	        
	        //boundary following behavior!
	        if(dist > last_dist)
	    	{
	    		dReached = pose_xy::getLinearDistance(goal.pose.p, robot.pose.p);
	    		dFollowp = robot.laser.findClosestTangentPoint(goal.pose.p, robot.pose.p);
	    		dFollow = pose_xy::getLinearDistance(dFollowp, goal.pose.p);
	    		while(dReached >= dFollow)
	    		{
	    			ros::spinOnce();
	    			rate.sleep();

	    			goalRobotV = Controller::getRobotGoalVector(dFollow, robot.pose.p);
	        		robot.move(goalRobotV.x, goalRobotV.y);
	    		}
	    	}
	        else
	        {
	        	goalRobotV = Controller::getRobotGoalVector(goal.pose.p, robot.pose.p);
	        	robot.move(goalRobotV.x, goalRobotV.y);
	        }
	        ros::spinOnce();
    	    dist = pose_xy::getLinearDistance(goal.pose.p, Oi) + pose_xy::getLinearDistance(Oi, robot.pose.p);
	        last_dist = dist;
	    }
    } 
    return EXIT_SUCCESS; 
}
