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

class Base : public pose_xy{ 
public: 
    pose_xy p; 

    Base(){}

    void baseCallback(const nav_msgs::Odometry::ConstPtr& msg) { 
        double roll, pitch; 
        x = msg->pose.pose.position.x; 
        y = msg->pose.pose.position.y; 
        //printf("Getting base!!!! %f %f\n", x, y);
        tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x, \
                                          msg->pose.pose.orientation.y, \
                                          msg->pose.pose.orientation.z, \
                                          msg->pose.pose.orientation.w); 
        tf::Matrix3x3(q).getRPY(roll, pitch, heading); 
    } 
}; 

class Pose : public pose_xy{ 
public: 

    Pose() {} 
    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) { 
        double roll, pitch; 
        x = msg->pose.pose.position.x; 
        y = msg->pose.pose.position.y;
        //printf("Getting pose!!!! %f %f\n", x, y); 

        tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x, \
                                          msg->pose.pose.orientation.y, \
                                          msg->pose.pose.orientation.z, \
                                          msg->pose.pose.orientation.w); 
        tf::Matrix3x3(q).getRPY(roll, pitch, heading); 
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

    pose_xy laserIndexToPose(int i, double dist, double heading, pose_xy robot_pose)
    {
        double ang_point = heading - (laser.angle_increment * i + laser.angle_min);
        pose_xy point;
        point.x = robot_pose.x + dist*cos(ang_point);
        point.y = robot_pose.y + dist*sin(ang_point);
        point.heading = ang_point;

        return point;
    }

    pose_xy getMinimumDistanceRobotToPoint (pose_xy dummy_pose, pose_xy robot_pose)
    {
        float minimum = laser.range_max;
        int min_index = 0;
        pose_xy point;

        //printf("range: %1.2f \t laser.range.size: %lu\n", -1.23,laser.ranges.size());
        for(int i = 0; i < laser.ranges.size(); i++)
        {
            //ROS_INFO("Dentro do for");
            if (laser.ranges[i] > laser.range_min && laser.ranges[i] < minimum)
            {
                minimum = laser.ranges[i];
                min_index = i;
                // printf("range: %1.2f \t min_index: %2d\n", minimum,min_index);
                //ROS_INFO("Dentro do IF");
            }
        }

        double ang_point = robot_pose.heading - (laser.angle_increment * min_index + laser.angle_min);

        // point.x = robot_pose.x + minimum*cos(ang_point);
        // point.y = robot_pose.y + minimum*sin(ang_point);
        // ROS_INFO("Laser method");
        point.x = minimum*cos(ang_point);
        point.y = minimum*sin(ang_point);
        point.heading = ang_point;
            

        //point = laserIndexToPose(min_index, minimum, robot_pose.heading, robot_pose);
        point.x += robot_pose.x;
        point.y += robot_pose.y;

        //printf("closest point:%f %f - distance: %f\n robot_x: %f, robot_y: %f\n\n", point.x, point.y, minimum, robot_pose.x, robot_pose.y);
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
	    	double ang_point, dist = HUGE_VAL;
	    	double minDist = HUGE_VAL;
	    	pose_xy minPoint, point;
	    	bool onObstacle = false;

            if(laser.ranges[0] > laser.range_min && laser.ranges[0] < laser.range_max)
            {
                laserDist = laser.ranges[0];

                point = laserIndexToPose(0, laserDist, robot_pose.heading, robot_pose);

                dist = pose_xy::getLinearDistance(goal, point) +  pose_xy::getLinearDistance(point, robot_pose);
                //printf("dist %f\n", dist);
                if(dist < minDist)
                {
                    minPoint = point;
                    minDist = dist;
                   // printf("changed point");
                }    

                onObstacle = true;
            }    

	    	for(int i = 0; i < laser.ranges.size(); i++)
	    	{
	    		if (laser.ranges[i] > laser.range_min && laser.ranges[i] < laser.range_max)
                {
	    			if(onObstacle == false)
	    			{
		    			laserDist = laser.ranges[i];
		    			
		    			point = laserIndexToPose(i, laserDist, robot_pose.heading, robot_pose);

				    	dist = pose_xy::getLinearDistance(goal, point) +  pose_xy::getLinearDistance(point, robot_pose);
				    	if(dist < minDist)
                        {
                            minPoint = point;
                            //printf("px: %f  py: %f\n", point.x, point.y);
                        }
				    		

                        onObstacle = true;
				    }
                else if (onObstacle == true)
                {
                    laserDist = laser.ranges[i];
                        
                    point = laserIndexToPose(i, laserDist, robot_pose.heading, robot_pose);

                    dist = pose_xy::getLinearDistance(goal, point) +  pose_xy::getLinearDistance(point, robot_pose);
                    if(dist < minDist)
                    {
                        minPoint = point;
                        //printf("px: %f  py: %f\n", point.x, point.y);
                    }
                        

                    onObstacle = false;
                }
	    		}
                else
                {
                    onObstacle = false;
                }
	    	}

            if (minDist == HUGE_VAL)
                minPoint = goal;
            //printf("%f %f \n", minPoint.x, minPoint.y);
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

        /*v = kv*( cos(pose.heading)*xvel + sin(pose.heading)*yvel );
        w = kw*( -sin(pose.heading)*yvel/d + cos(pose.heading)*yvel/d );
        */
        double angVelz = atan2(yvel, xvel) - pose.heading; 
        double distance = sqrt(xvel*xvel + yvel*yvel); 
        if (std::abs(angVelz) > 1e-4){ 
            v = 0;
            w = 3* angVelz; 
        } 
        else { 
            v = distance * 0.8;
            w = 0; 
        } 

        printf("\ndx %f dy %f --- v %f w %f\n", xvel, yvel, v, w);

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

    static double getAngularDistance(StageBot& robot, StageBot& goal) 
    { 
        return atan2(goal.pose.y - robot.pose.y, \
                     goal.pose.x - robot.pose.x) \
                   -robot.pose.heading; 
    } 

    static double getLinearDistance(StageBot& robot, StageBot& goal) 
    { 
        return sqrt(pow(goal.pose.x - robot.pose.x, 2) \
                    + pow(goal.pose.y - robot.pose.y, 2)); 
    }

    static double getLinearDistance(pose_xy p1, pose_xy p2) 
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
    StageBot robot(n, 0, 0.1, 0.2, 1); 
    StageBot goal(n, 1); 
    Controller controller; 

    double K = 1; 
    pose_xy Oi, goalRobotV;
    double dist;
    double last_dist=HUGE_VAL;

    double dReached, dFollow;
    pose_xy dFollowp;

    ros::Rate rate(20); 

    ros::Duration(0.5).sleep();

    while (ros::ok()) { 
    	rate.sleep();
    	ros::spinOnce();
		
        //Oi = robot.laser.getMinimumDistanceRobotToPoint(goal.base, robot.base);
        //Oi = robot.laser.findClosestTangentPoint(goal.base, robot.base);

    	if (pose_xy::getLinearDistance(goal.base, robot.base) < 0.1)
    	{
    		ROS_INFO("GOAL REACHED!");
    		printf("goal->  x: %1.2f \t y: %1.2f\n robot-> x: %1.2f \t y: %1.2f\n", goal.base.x, goal.base.y, robot.base.x, robot.base.y);
        	robot.move(0, 0);

    		return EXIT_SUCCESS;
		}
		else
		{
	        if(robot.laser.obstacle_in_path(goal.base, robot.base))
	        {
                printf("there is an obstacle on the path\n");
	        	Oi = robot.laser.findClosestTangentPoint(goal.base, robot.base);
	        }
	        else
	      	{
                printf("no obstacle!\n");
	      		Oi = goal.base;
                goalRobotV = Controller::getRobotGoalVector(Oi, robot.base);
                robot.move(goalRobotV.x, goalRobotV.y);
	      	}

	        dist = pose_xy::getLinearDistance(goal.base, Oi) + pose_xy::getLinearDistance(Oi, robot.base);
	        
	        //boundary following behavior!
	        if(dist > last_dist)
	    	{
	    		dReached = pose_xy::getLinearDistance(goal.base, robot.base);
	    		dFollowp = robot.laser.findClosestTangentPoint(goal.base, robot.base);
	    		dFollow = pose_xy::getLinearDistance(dFollowp, goal.base);
	    		while(dReached >= dFollow)
	    		{
	    			rate.sleep();
	    			ros::spinOnce();
	    			

	    			goalRobotV = Controller::getRobotGoalVector(dFollow, robot.base);
	        		robot.move(goalRobotV.x, goalRobotV.y);
	    		}
	    	}
	        else
	        {
	        	goalRobotV = Controller::getRobotGoalVector(Oi, robot.base);
	        	robot.move(goalRobotV.x, goalRobotV.y);
	        }
            


	        ros::spinOnce();
    	    dist = pose_xy::getLinearDistance(goal.base, Oi) + pose_xy::getLinearDistance(Oi, robot.base);
	        last_dist = dist;
	    }
    } 
    return EXIT_SUCCESS; 
}
