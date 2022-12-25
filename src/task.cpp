#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros/console.h>
#include "geometry_msgs/Twist.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include "ar_track_alvar_msgs/AlvarMarkers.h"
#include <cmath>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/Odometry.h"
#include <cstdlib>
#include "actionlib_msgs/GoalStatusArray.h"
#include <vector>

#define PI 3.142857
ros::Publisher speed_pub; // declare global publisher to allow its definition to use the node handler as well as its usage in the service callback func
ros::Publisher target_pub;
geometry_msgs::PoseStamped target;
geometry_msgs::PoseStamped navGoal;
geometry_msgs::PoseStamped finalGoal;
int id;
int old_id;
float dist;
//bool static;
float distThre;
double roll,pitch,yaw;
double posx,posy,posz;
float last_dist;
nav_msgs::Odometry crtPose;
int count;
bool scaleTarget;
bool justScale;
geometry_msgs::Twist msg;
int state;
std::vector<int> hist;
bool odd;
bool restart;
int turn_count;
bool acrossRoom_lock;
int last; // used when the target is lost and restart
void callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& ar_pose); 
void status_callback(const actionlib_msgs::GoalStatusArray::ConstPtr& status);
void transformPoint(const tf2_ros::TransformListener& listener);
void turn_around();
void odom_callback(const nav_msgs::Odometry& odom);
float getTurnRadian(float crtYaw, float rangeA, float rangeB);
int reached(int id);
void passObstacle();

int reached(int idd){
	for (int i = 0; i < hist.size(); i++){
		if (hist[i] == idd)
			return i;
	}
	return -1;
}
void passObstacle(){
	msg.linear.y = 0;
	msg.linear.z = 0;
	msg.angular.x = 0;
	msg.angular.y = 0;	
	msg.angular.z = 0;	
	msg.linear.x = -1;
	speed_pub.publish(msg);
	ros::Duration(1).sleep();
	msg.linear.x = 0;
	msg.angular.z = pow(-1,odd)*PI/2;
	speed_pub.publish(msg);
	ros::Duration(1).sleep();
	msg.angular.z = 0;
	speed_pub.publish(msg);
	target_pub.publish(finalGoal);
	odd = 1-odd;
	ROS_INFO("Fail.Move_back");
}
float getTurnRadian(float crtYaw, float rangeL, float rangeR){
	float planA = rangeL - crtYaw;
	float planB = rangeR - crtYaw;
	float plan;
	if (planA > PI)
		planA -= 2*PI;
	else if (planA < -PI)
		planA += 2*PI;
	if (planB > PI)
		planB -= 2*PI;
	else if (planB < -PI)
		planB += 2*PI;

	if (abs(planA) > abs(planB))
		plan = planB;
	else
		plan = planA;
	if (plan < 0)
		plan =  (plan > -0.6) ? -0.2 : (plan+0.6);
	else
		plan =  (plan < 0.6) ? 0.2 : (plan-0.6);
	return plan;
}	

void turn_around(){
	if ((state == 0) || (state == 1) || (state == 6) || (state == 7)){
		ROS_INFO("WAIT FOR move_base to be ready (crtState=%d)",state);
	}
	else{
		if (id != -1)
			old_id = id; 
		id = -1;
		hist.push_back(old_id);

		ROS_INFO("Start to turn around to search nxt tag. Last id:%d, turn_count=%d",old_id,turn_count);
		// initiate the speeds
		msg.linear.x = 0;
		msg.linear.y = 0;
		msg.linear.z = 0;
		msg.angular.x = 0;
		msg.angular.y = 0;	
		// turn
		// turn to the correct range
		switch (old_id)
		{
			case 0:
				msg.angular.z = getTurnRadian(yaw,PI/4,0);
				break;
			case 7:
				msg.angular.z = getTurnRadian(yaw,0,-PI/4);
				break;
			case 6:
				msg.angular.z = getTurnRadian(yaw,-PI/4,-PI/2);
				break;
			case 3:
				msg.angular.z = getTurnRadian(yaw,3*PI/4,PI);
				break;
			case 4:
				msg.angular.z = getTurnRadian(yaw,PI,-3*PI/4);
				break;
			case 5:
				msg.angular.z = getTurnRadian(yaw,-PI/2,-3*PI/4);
				break;
			case 1:
				msg.angular.z = getTurnRadian(yaw,PI/4,PI/2);
				break;
			case 2:
				msg.angular.z = getTurnRadian(yaw,PI/2,3*PI/4);
			default:
				msg.angular.z = 0;
				break;
		}
		msg.angular.z /= 2;
		if (old_id == 8)
			return;
		if (abs(msg.angular.z) > 0.1){
			speed_pub.publish(msg);
			ROS_INFO("Start turning with z = %f",msg.angular.z);
			ros::Duration(2).sleep();
		}
		// start searching for next ar
		if (msg.angular.z >= 0)
			msg.angular.z = 0.1;
		else if (msg.angular.z < 0)
			msg.angular.z = -0.1;
		speed_pub.publish(msg);
		ROS_INFO("Start turning with z = %f",msg.angular.z);
	}
}

void status_callback(const actionlib_msgs::GoalStatusArray::ConstPtr& stat){
	if ((state == 0) || (state == 1) || (state == 6) || (state == 7))
		turn_count = 0;
	else if (dist < distThre){
		if (acrossRoom_lock){
			//distThre = 4;
			if (state == 3){
				ROS_INFO("Release the lock with state %d",state);
				msg.linear.x= 2;
				speed_pub.publish(msg);
				ros::Duration(1).sleep();
				msg.linear.x = 0;
				speed_pub.publish(msg);	
				ROS_INFO("rush into the room");		
				acrossRoom_lock = 0;
			}
			else{
				passObstacle();
			}
		}
	}
	if ((stat->status_list.size() != 0) && (old_id != 8)){
		state = stat->status_list[stat->status_list.size()-1].status;
		ROS_INFO("move_base status=%d",state);
		if (((state == 4) || (state == 5) || (state == 9)) && (dist > distThre)){
			passObstacle();
		}
		if ((state == 3) && (dist > distThre)){
			// lose the target. RESTART
			if (!restart){
				last = id;
				id = old_id;
				turn_around();
				ROS_INFO("Lose target. Restart.");
				restart = 1;
				//target_pub.publish(finalGoal);
			}
			else{
				ROS_INFO("Have restarted before. Still searching %d...", last);
			}
		}
	}
}

void odom_callback(const nav_msgs::Odometry& odom){
	tf::Quaternion quat;
	tf::quaternionMsgToTF(odom.pose.pose.orientation,quat);
	tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
	last_dist = dist;
	dist =  sqrt(pow(odom.pose.pose.position.x-finalGoal.pose.position.x,2)+pow(odom.pose.pose.position.y-finalGoal.pose.position.y,2)+pow(odom.pose.pose.position.z-finalGoal.pose.position.z,2));
	// if (dist > (distThre + 2))
	// 	distThre = 11;
	ROS_INFO("dist=%f, YAW=%f, distThre=%f",dist,yaw,distThre);
	posx = odom.pose.pose.position.x;
	posy = odom.pose.pose.position.y;
	posz = odom.pose.pose.position.z;
}

void callback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& ar_pose)
{
	// set loop frequency
	ros::Rate loop_rate(1);
	if ((reached(8) == -1) && (acrossRoom_lock == 0)){
		ROS_INFO("AR callback: target %d", id);	
		// the first tag
		if ((id == -2) && (ar_pose->markers.size()!=0))
			id = ar_pose->markers[0].id;
		// initiate the speeds
		msg.linear.x = 0;
		msg.linear.y = 0;
		msg.linear.z = 0;
		msg.angular.x = 0;
		msg.angular.y = 0;	
		msg.angular.z = 0;	
		// if (id == 4){
		// 	msg.linear.x= 0.5;
		// 	speed_pub.publish(msg);
		// 	ros::Duration(1).sleep();
		// 	msg.linear.x = 0;
		// 	speed_pub.publish(msg);	
		// 	ROS_INFO("0.5m first");			
		// }	
		if ((state != 3) || (dist <= distThre) || (last != -1)){
			ROS_INFO("judge...");
			// 1. no ar detectet
			if  (ar_pose->markers.size()==0){
				count = 0;
				turn_count++;
				if (justScale){
					scaleTarget = 1-scaleTarget;
					justScale = 0;
				}
				ROS_INFO("no ar detected");
				ROS_INFO("id = %d, oid = %d, dist=%f", id, old_id,dist);
				if ((dist < distThre) && (state != 0) && (state != 1) && (state != 6) && (state != 7)){
					ROS_INFO("turn around");		
					turn_around();
					if (turn_count > 23){ // turn for a round
						double doorx2 = -4.86;
						double doory2 = -3.79;

						double doorx1 = 8.2;
						double doory1 = -3.79;
						double d1 = sqrt(pow(doorx1-posx,2)+pow(doory1-posy,2));
						double d2 = sqrt(pow(doorx2-posx,2)+pow(doory2-posy,2));
						//if (d1 < d2){
							navGoal.pose.position.x = doorx1;
							navGoal.pose.position.y = doory1;
							navGoal.pose.orientation.z = -0.20130226363463208;
							navGoal.pose.orientation.w = 0.979529171926785;
							ROS_INFO("Cannot see the target. Move to the door left first.");
						/*}
						else{
							navGoal.pose.position.x = doorx2;
							navGoal.pose.position.y = doory2;
							navGoal.pose.orientation.z = -0.2590263401609905;
							navGoal.pose.orientation.w = 0.9658702579036187;	
							ROS_INFO("Cannot see the target. Move to the door right first.");					
						}*/
						navGoal.pose.orientation.x = 0;
						navGoal.pose.orientation.y = 0;
						ros::param::set("/move_base/DWAPlannerROS/xy_goal_tolerance",0.5);
						ros::param::set("/move_base/DWAPlannerROS/occdist_scale",3);
						target_pub.publish(navGoal);
						finalGoal = navGoal;
						turn_count = 0;
						acrossRoom_lock = 1; // lock this process
						ros::Duration(10).sleep();
					}
				}
				else{
					ROS_INFO("is still chasing");
					//target_pub(finalGoal);
					//msg.angular.z = 0.1;
					//speed_pub.publish(msg);
				}
			}
			
			else {
				ROS_INFO("!!!");
				// 2. get id different from the current id record
				if (ar_pose->markers[0].id != id){
					count = 0;
					if (justScale){
						scaleTarget = 1-scaleTarget;
						justScale = 0;
					}
					ROS_INFO("dif id");
					if ((id == -1) && (reached(ar_pose->markers[0].id)==-1)){ // find the next id
						// stop
						msg.angular.z = 0;
						speed_pub.publish(msg);
						ROS_INFO("NEXT!");
						//if ((last == -1) || (ar_pose->markers[0].id == last)){
						if (1){
							turn_count = 0;
							ROS_INFO("find the next id %d",ar_pose->markers[0].id);
							ROS_INFO("id = %d, oid = %d, last = %d", id, old_id, last);
							scaleTarget = 0;

							id = ar_pose->markers[0].id;

							// set the navigation goal
							target.header = ar_pose->header;
							target.pose = ar_pose->markers[0].pose.pose;
							target.header.frame_id = "virtual_camera_link";
							//ROS_INFO(target.header.frame_id);
							// transform the frame
							//tf2_ros::Buffer *tfBuffer = new tf2_ros::Buffer;
							tf2_ros::Buffer tfBuffer;
							tf2_ros::TransformListener listener(tfBuffer);
							//transformPose(listener,tfBuffer);
							while (1){
								if (tfBuffer.canTransform("map","virtual_camera_link",ros::Time(), ros::Duration(10.0))){
									try{
										//tf::StampedTransform transform;
										geometry_msgs::TransformStamped transform;
										//listener.waitForTransform("odom", "virtual_camera_link", ros::Time::now(), ros::Duration(10.0));
										transform = tfBuffer.lookupTransform("map","virtual_camera_link",ros::Time(0), ros::Duration(1.0));
										//ros::Duration(1).sleep();
										//listener.transformPose("odom", target, navGoal);
										tf2::doTransform(target,navGoal,transform);

										ROS_INFO("virtual_camera_link: (%.2f, %.2f. %.2f) -----> map: (%.2f, %.2f, %.2f) at time %.2f",
											target.pose.position.x, target.pose.position.y, target.pose.position.z,
											navGoal.pose.position.x, navGoal.pose.position.y, navGoal.pose.position.z, navGoal.header.stamp.toSec());
										//delete tfBuffer;
										break;
									}
									catch(tf::TransformException& ex){
										ROS_ERROR("Received an exception trying to transform a point from \"virtual_camera_link\" to \"map\": %s", ex.what());
										id = -1;
									}
								}
							}
							ros::param::set("/move_base/DWAPlannerROS/xy_goal_tolerance",3);
							ros::param::set("/move_base/DWAPlannerROS/occdist_scale",2);
							// publish the navigation goal
							target_pub.publish(navGoal);
							finalGoal = navGoal;
							last = -1;
							restart = 0;
							ros::Duration(1).sleep();
							//distThre = dist - 2;
							/*distThre = dist * 0.85;
							if (distThre < 8.5)
								distThre = 8.5;*/
						}
					}	
					else if ((id == -1) && (ar_pose->markers[0].id == old_id)){
						ROS_INFO("SEARCH for the next id");
						ROS_INFO("id = %d, oid = %d", id, old_id);
						turn_around();
					}
					if (turn_count != 0){
						turn_count++;
					}
				}
				// 3. get id same as the current id record
				else if (ar_pose->markers[0].id == id){
					ROS_INFO("same id");
					ROS_INFO("id = %d, oid = %d", id, old_id);
					ROS_INFO("distance = %f",dist);
					ROS_INFO("count = %d",count);
					
					if ((dist < distThre) && (state != 0) && (state != 1) && (state != 6) && (state != 7)){
						count = 0;
						ROS_INFO("no need to count");
						if (justScale){
							scaleTarget = 1-scaleTarget;
							justScale = 0;
						}
						turn_around();
					}
					else if ((abs(last_dist-dist)<0.001)){
						count += 1;
						ROS_INFO("increase count");
						if (count >= 5){
							if ((state != 3) && ((scaleTarget == 0) || (count >= 5))){
								msg.linear.x = -0.5;
								speed_pub.publish(msg);
								ros::Duration(1).sleep();
								msg.linear.x = 0;
								msg.angular.z = pow(-1,odd)*PI/2;
								speed_pub.publish(msg);
								ros::Duration(1).sleep();
								msg.angular.z = 0;
								speed_pub.publish(msg);
								odd = 1-odd;

								float scale = rand()/double(RAND_MAX);
								navGoal.pose.position.x = finalGoal.pose.position.x*scale;
								navGoal.pose.position.y = finalGoal.pose.position.y*scale;
								target_pub.publish(navGoal);
								justScale = 1;
								ROS_INFO("RESET navGoal with %f",scale);
								ros::Duration(10).sleep();
							}
							else{
								scaleTarget = 1;
								count = 6;
								count = 0;
								target_pub.publish(finalGoal);
								justScale = 1;
								ros::Duration(10).sleep();
								ROS_INFO("RESET navGoal with the original goal");
							}
						}
					}
				}
			}
		}
		// Loop waiting for the callback function
		ros::spinOnce;
		// delay based on the loop frequency
		loop_rate.sleep();
	}
}


int main(int argc, char **argv)
{
 id = -2;
 old_id = -1;
 acrossRoom_lock = 0;
 dist = 0;
 distThre = 6;
 scaleTarget = 0;
 justScale = 0;
 //static = 1;
 state = -1;
 hist = {100};
 last = -1;
 odd = 0;
 restart = 0;
 turn_count = 0;
 navGoal.header.frame_id = "map";
 // ROS node initialization
 ros::init(argc, argv, "follower_ctrl");
 // create node handler
 ros::NodeHandle n; 
 ros::param::set("/use_sim_time", "true");
  // Create a Publisher, publish a topic named /cmd_vel, from which the spark receives the speed command 
 // queue length is 0 (only the latest)
 target_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
 speed_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
 // Create a subscriber of the topic of face position with "callback" as the callback func to react to the position info.
 // queue length is 0(the latest)
 ros::Subscriber pose_sub = n.subscribe<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker", 1, callback);
 ros::Subscriber odom_sub = n.subscribe("/odom", 1, odom_callback);
 ros::Subscriber status_sub = n.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 1, status_callback);
 // sleep for 2s, the time for system initiation. Without this command, some following commands sometimes may be ignored.
 ros::Duration(2).sleep();
 // Inform the programmer that the program has been ready.
 ROS_INFO("PREPARE: Ready to run.");
 // Loop waiting for the callback function
 ros::spin();

 return 0;
}

