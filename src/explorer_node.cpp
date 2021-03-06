/*
 * Standard libs include
 */
#include <iostream>
#include <float.h>
#include <time.h>
#include <cmath>
#include <string>


/*
 * Ros includes
 */
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"

/*
 * OpenCV includes
 */
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/videoio/videoio.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/*
 * node's libraries
 */
#include "rrt_path_finder.hpp"


#define ROBOT_RADIUS 0.35
#define OP_FACTOR 3


#define ROBOT_RADIUS 0.35

double targetX=0;
double targetY=0;

typedef struct _Robot
{
	// x, y, theta
	double robot_pos[3];
	double robot_pos_in_image[2];
}Robot;

typedef struct _Map
{
	// int8_t *data;
	cv::Mat map;
	uint32_t height;
	uint32_t width;
	float res;
	double origin[2];
	double tX;
	double tY;
}Map;





Robot r;

void odom_to_map(Robot& r, const Map& m)
{
	r.robot_pos_in_image[0] = r.robot_pos[0]/m.res - m.origin[0]/m.res;
	r.robot_pos_in_image[1] = m.height - (r.robot_pos[1]/m.res - m.origin[1]/m.res);
}

Vertex *map_to_odom(const Vertex *v, const Map& m)
{
	Vertex *ret = new Vertex();
	ret->data[0] = v->data[0]*m.res + m.origin[0];
	ret->data[1] = -(v->data[1] - m.height)*m.res + m.origin[1];
	return ret;
}

static void onMouse( int event, int x, int y, int, void* )
{
	if( event != cv::EVENT_LBUTTONDOWN )
		return;
	targetX= x;
	targetY= y;

	std::cout << " x:" << x << "  y:" << y << std::endl;
}

bool get_slam_map(Map& map)
{
	ros::NodeHandle n;

  	ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("dynamic_map");
	nav_msgs::GetMap srv;
	if (client.call(srv))
		{
			//ROS_INFO("Service GetMap succeeded.");
			std_msgs::Header header = srv.response.map.header;
			nav_msgs::MapMetaData info = srv.response.map.info;
			map.width = info.width;
			map.height = info.height;
			map.res = info.resolution;
			map.origin[0] = info.origin.position.x;
			map.origin[1] = info.origin.position.y;
			map.tX = targetX;
			map.tY = targetY;
			if(map.height > 0 && map.width > 0)
				{
					cv::Mat tmp(map.height, map.width, CV_8UC1, &(srv.response.map.data[0]));
					cv::Mat se_ouverture = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(ROBOT_RADIUS/map.res/OP_FACTOR<1?1:ROBOT_RADIUS/map.res/OP_FACTOR, ROBOT_RADIUS/map.res/OP_FACTOR<1?1:ROBOT_RADIUS/map.res/OP_FACTOR),cv::Point(-1,-1));
					cv::Mat se = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(3*ROBOT_RADIUS/map.res<1?1:ROBOT_RADIUS/map.res, 3*ROBOT_RADIUS/map.res<1?1:ROBOT_RADIUS/map.res),cv::Point(-1,-1));
					// Ouverture pour supprimer les petits elements et grossir les gros
					cv::dilate(tmp, tmp, se_ouverture, cv::Point(-1,-1), 1, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());
					cv::erode(tmp, tmp, se_ouverture, cv::Point(-1,-1), 1, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());
					//cv::erode(emptyMap, emptyMap, se, cv::Point(-1,-1), 3, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());
					cvtColor(tmp,map.map,CV_GRAY2RGB);
					//tmp.copyTo(map.map);// = cv::Mat(map.height, map.width, CV_8UC1, &(srv.response.map.data[0]));  
					cv::Mat mask;
					cv::inRange(map.map, cv::Scalar(50,50,50), cv::Scalar(250,250,250), mask);
					map.map.setTo(cv::Scalar(0,255,0), mask);
					cv::bitwise_not(map.map, map.map, cv::noArray());

					//cv::threshold(map.map, map.map, 254, 255, cv::THRESH_BINARY);

					flip(map.map, map.map, 0);

					odom_to_map(r, map);
				}

			//ROS_INFO("Map width, height: %u, %u", map.width, map.height);
		}
	else
		{
			ROS_ERROR("Service GetMap failed.");
			return false;
		}
	ROS_INFO("Map loading succeeded.");
	return true;
}



/* Main fonction */
int main(int argc, char* argv[])
{

	
	ros::init(argc, argv, "explorer_node");
	ros::NodeHandle n;
	ros::Rate loop_rate(100); //10 Hz
	ros::Publisher pubGoal = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
	geometry_msgs::PoseStamped goal;
	

	bool exploring=true;

	// /* Real part, not to show off ! */
	Map map;
	map.width = 0;
	map.height = 0;
	targetX = -1;
	targetY = -1;
	double targetPastX = -1;
	double targetPastY = -1;
	if(!get_slam_map(map))	{ return 1;}
	tf::TransformListener t;
    
	r.robot_pos[0] = -1;
	r.robot_pos[1] = -1;

	
	while(ros::ok() && r.robot_pos[0] == -1 && r.robot_pos[1] == -1)
		{
			// Get frame change between slam_karto map frame and the frame of the odom of the robot
			tf::StampedTransform transform_slam;
			try
				{
					t.lookupTransform("map", "base_link", ros::Time(0), transform_slam);
				}
			catch (tf::TransformException ex)
				{
					ROS_ERROR("%s",ex.what());
					ros::Duration(1.0).sleep();
				}
			r.robot_pos[0] = transform_slam.getOrigin().x();
			r.robot_pos[1] = transform_slam.getOrigin().y();
			goal.pose.position.x= r.robot_pos[0] ;
			goal.pose.position.y= r.robot_pos[1] ;

			odom_to_map(r, map);
		}

	while(ros::ok())
		{
			ros::spinOnce();
			// Get frame change between slam_karto map frame and the frame of the odom of the robot
			tf::StampedTransform transform_slam;
			try
				{
					t.lookupTransform("map", "base_link", ros::Time(0), transform_slam);
				}
			catch (tf::TransformException ex)
				{
					ROS_ERROR("%s",ex.what());
					ros::Duration(1.0).sleep();
				}
			get_slam_map(map);
			r.robot_pos[0] = transform_slam.getOrigin().x();
			r.robot_pos[1] = transform_slam.getOrigin().y();

			odom_to_map(r, map);
		
			// if(targetY != targetPastY && targetX != targetPastX)
			// 	{
			// 		if(rviz_goal_flag)
			// 		{
			// 			targetX = targetX/map.res - map.origin[0]/map.res;
			// 			targetY = map.height - (targetY/map.res - map.origin[1]/map.res);
			// 			std::cout << targetX << " " << targetY << std::endl;
			// 			rviz_goal_flag = false;
			// 		}
			// 		std::cout << "Robot REAL pos: " << r.robot_pos[0] << " | " << r.robot_pos[1] << std::endl;
			// 		std::cout << "Robot IM pos: " << r.robot_pos_in_image[0] << " | " << r.robot_pos_in_image[1] << std::endl;
			// 		path = rrt(new Vertex{{r.robot_pos_in_image[0],r.robot_pos_in_image[1]},NULL,0,0}, map, with_gui);
			// 		path_to_pub = construct_path_msg(path, map);
			// 		targetPastX = targetX;
			// 		targetPastY = targetY;
			// 	}

			if(1)
			{
				cv::circle(map.map, cv::Point(r.robot_pos_in_image[0], r.robot_pos_in_image[1]), ROBOT_RADIUS/map.res, cv::Scalar(255,0,0), -1, 8, 0);
				double reach_zone=12;
				std::cout << abs(goal.pose.position.x - r.robot_pos_in_image[0]) << "  "<< abs(goal.pose.position.y - r.robot_pos_in_image[1]) << std::endl; 
				if(exploring && abs(goal.pose.position.x - r.robot_pos[0]) < reach_zone && abs(goal.pose.position.y - r.robot_pos[1]) < reach_zone)
					{
						Vertex qrand;
						Vertex qnear =  Vertex{{r.robot_pos_in_image[0],r.robot_pos_in_image[1]},NULL,0,0};
						Vertex*	qnew = NULL;
						double dq = MAX_INC;
						while(qnew == NULL)
							{
								rand_free_conf(qrand, map.height, map.width);
								qnew = new_conf(qrand, qnear, &dq,  map.map);
								std::cout << dq << std::endl;
								dq = MAX_INC;
							}
						targetX/map.res - map.origin[0]/map.res;
						targetY = map.height - (targetY/map.res - map.origin[1]/map.res);

						goal.pose.position.x = qnew->data[0]*map.res + map.origin[0];
						goal.pose.position.y = (map.height-qnew->data[1])*map.res + map.origin[1];
						pubGoal.publish(goal);
					}
				cv::circle(map.map, cv::Point(goal.pose.position.x ,goal.pose.position.y ), 2, cv::Scalar(0,255,0), -1, 8, 0);
				cv::imshow( "Display window", map.map );                   // Show our image inside it.
				cv::circle(map.map, cv::Point(r.robot_pos_in_image[0], r.robot_pos_in_image[1]), ROBOT_RADIUS/map.res, cv::Scalar(255,0,0), -1, 8, 0);
			}

			// pubPath.publish(path_to_pub);
			cv::waitKey(10);
		}
	
 	return 0;
 }
