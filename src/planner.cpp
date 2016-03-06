#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>

#include <std_msgs/String.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <math.h> 
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <stdlib.h>

#include <actionlib/server/simple_action_server.h>
#include <planner/gen_trajAction.h>
using namespace std;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


class Trajectory
{
  int count;
  
public:
  Eigen::ArrayXf traj_x,traj_y,traj_theta,traj_t;
  visualization_msgs::Marker points, line_strip, line_list;
  float dt, t_end;

  Trajectory();
  
  void generate_trajectory(float,float);
  void straight_trajectory(float,float,float,float,float);
  void zero_pt_turn(float, float);
  void trasform_trajectory_to_current_pose(geometry_msgs::Pose);
  void init_markers();
  Eigen::ArrayXf traj_time_lookup(float);
};


Trajectory::Trajectory()
{

}

void Trajectory::init_markers()
{
  points = visualization_msgs::Marker();
  line_strip = visualization_msgs::Marker();
  line_list = visualization_msgs::Marker();

  points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/traj_frame";
  points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
  points.ns = line_strip.ns = line_list.ns = "points_and_lines";
  points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

  points.id = 0;
  line_strip.id = 1;
  line_list.id = 2;

  points.type = visualization_msgs::Marker::SPHERE;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_list.type = visualization_msgs::Marker::LINE_LIST;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.1;
  points.scale.y = 0.1;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.05;
  line_list.scale.x = 0.05;

  // Points are not green
  points.color.g = 1.0f;
  points.color.b = 1.0f;
  points.color.a = 1.0;

  // Line strip is blue
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  // Line list is red
  line_list.color.r = 1.0;
  line_list.color.b = 1.0;

  line_list.color.a = 1.0;
}

void Trajectory::generate_trajectory(float x,float y)
{

  init_markers();
  cout << "inited markers" << endl;

  float theta = atan2(y,x);
  cout << theta << "  " << x << " " << y << endl;
  
  float v = .4;
  float w = .3;
  dt = .5;
  
  float linear_t_end = sqrt(x*x + y*y)/v + dt;
  float angular_t_end = abs(theta)/w + dt;
  t_end = linear_t_end + angular_t_end;
  
  float t = 0;
  
  int length = int(1.2*t_end/dt); // THIS IS HACK PLS FIX
  cout << length << endl;
  traj_x = x + Eigen::ArrayXf::Zero(length);
  traj_y = y + Eigen::ArrayXf::Zero(length);
  traj_t = t_end + Eigen::ArrayXf::Zero(length);
  traj_theta = theta + Eigen::ArrayXf::Zero(length);
  

  count = 0;
  
  cout << "generating trajectories" << endl;
  zero_pt_turn(theta,w);
  cout << "x:\n" <<traj_x << "\ny:\n" << traj_y << "\ntheta:\n" << traj_theta << "\nt:\n" << traj_t << "\n DONE \n" << endl;
  straight_trajectory(x,y,theta,v,w);
  cout << "x:\n" <<traj_x << "\ny:\n" << traj_y << "\ntheta:\n" << traj_theta << "\nt:\n" << traj_t << "\n DONE \n" << endl;

}

// goal x, goal, y, goal theta, max v, max w

void Trajectory::straight_trajectory(float x,float y, float theta, float v, float w)
{
  // float theta = atan2(y,x);
  float linear_t_end = sqrt(x*x + y*y)/v;
  geometry_msgs::Point p;
  // cout << x << " " << y << " " << " " << theta << endl;
  float vx = x/linear_t_end;
  float vy = y/linear_t_end;
  float start_time = 0;
  if(count>0)
  {
    start_time = traj_t(count-1);  
  }
  float t = 0;

  while(t<linear_t_end)
  {
    p.x = vx*t;
    p.y = vy*t;
    p.z = 0;
    points.points.push_back(p);
    line_strip.points.push_back(p);
    line_list.points.push_back(p);
    traj_x(count) = (p.x);
    traj_y(count) = (p.y);
    traj_theta(count) = theta;
    traj_t(count) = (t) + start_time;

    p.x += vx/10;
    p.y += vy/10;
    line_list.points.push_back(p);
    t += dt;  
    count++;
  }
  p.x = x;
  p.y = y;
  p.z = 0;
  t = linear_t_end;
  points.points.push_back(p);
  line_strip.points.push_back(p);
  line_list.points.push_back(p);
  traj_x(count) = (p.x);
  traj_y(count) = (p.y);
  traj_theta(count) = (theta);
  traj_t(count) = (t) + start_time;
  p.x += vx;
  p.y += vy;
  line_list.points.push_back(p);
  count++;
  
  return;
}

void Trajectory::zero_pt_turn(float theta, float w)
{
  float angular_t_end = abs(theta)/w;
  float t = 0;
  float start_time = 0;
  if(count>0)
  {
    start_time = traj_t(count-1);  
  }
  geometry_msgs::Point p;
  while(t<angular_t_end)
  {
    
    p.x = 0;
    p.y = 0;
    p.z = 0;
    points.points.push_back(p);
    line_strip.points.push_back(p);
    line_list.points.push_back(p);
    traj_x(count) = (p.x);
    traj_y(count) = (p.y);
    traj_theta(count) = w*t*sgn(theta);
    traj_t(count) = (t) + start_time;

    p.x += w*cos(w*t*sgn(theta));
    p.y += w*sin(w*t*sgn(theta));
    line_list.points.push_back(p);
    t += dt;  
    count++;
  }
  p.x = 0;
  p.y = 0;
  p.z = 0;
  t = angular_t_end;
  points.points.push_back(p);
  line_strip.points.push_back(p);
  line_list.points.push_back(p);
  traj_x(count) = (p.x);
  traj_y(count) = (p.y);
  traj_theta(count) = (theta);
  traj_t(count) = (t) + start_time;

  p.x += cos(theta);
  p.y += sin(theta);
  line_list.points.push_back(p);
  count++;
}


class Trajectory_Tracker
{
protected:
  actionlib::SimpleActionServer<planner::gen_trajAction> action_server;
  string action_name;
  planner::gen_trajFeedback feedback;
  planner::gen_trajResult result;
  tf::TransformBroadcaster br;
  ros::Publisher marker_pub;
  tf::TransformListener listener;
  ros::Publisher cmd_pub;

public:
  
  geometry_msgs::PoseStamped current_pose;
  geometry_msgs::PoseStamped previous_pose;
  Trajectory current_trajectory;
  float Kp_x,Kp_y,Kp_w;

  Trajectory_Tracker(ros::NodeHandle n) :
    action_server(n,ros::this_node::getName(),boost::bind(&Trajectory_Tracker::track_trajectory, this, _1), false),
    action_name(ros::this_node::getName())
    {

      action_server.start();
      ROS_INFO("STARTED ACTION SERVER");
      marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
      cmd_pub = n.advertise<geometry_msgs::Pose2D>("cmd", 1000);
      Kp_x = 1;
      Kp_y = .75;
      Kp_w = 1;
    }

  void pose_callback(const geometry_msgs::PoseStamped::ConstPtr&);
  void track_trajectory(const planner::gen_trajGoalConstPtr &goal);
  geometry_msgs::Point transform_goal_to_robot(float, float);

  Eigen::ArrayXf get_error();
  tf::StampedTransform broadcast_new_traj_frame();

};

// Trajectory_Tracker::Trajectory_Tracker(string name)
// {
  // marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  // cmd_pub = n.advertise<geometry_msgs::Pose2D>("cmd", 1000);
  // Kp_x = 1;
  // Kp_y = .75;
  // Kp_w = 1;
  // action_name = = "planner_action";
// }

tf::StampedTransform Trajectory_Tracker::broadcast_new_traj_frame()
{
  tf::StampedTransform transform;
  try{
    listener.lookupTransform("/world", "/robot",  
                             ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/traj_frame"));
  return transform;
}

Eigen::ArrayXf Trajectory::traj_time_lookup(float t)
{
  int lower;
  int upper;
  float x,y,theta;
  if(t >= t_end){
    int last_index = traj_x.size()-1;
    float x = traj_x(last_index);
    float y = traj_y(last_index);
    float theta = traj_theta(last_index);
    cout << "past end time" << endl;
    Eigen::ArrayXf traj_at_t = Eigen::ArrayXf(4);
    traj_at_t << t,x,y,theta;
    return traj_at_t;

  }else if (t == 0)
  {
    float x = traj_x(0);
    float y = traj_y(0);
    float theta = traj_theta(0);
    cout << "start time" << endl;
    Eigen::ArrayXf traj_at_t = Eigen::ArrayXf(4);
    traj_at_t << t,x,y,theta;
    return traj_at_t;
  }else{
    for (int i = 0; i < traj_t.size(); ++i)
    {
      if(traj_t(i)>t)
      {
        lower = i-1;
        upper = i;
        break;
      }
    }

    float x = traj_x(lower) + (traj_x(upper) - traj_x(lower))*(t - traj_t(lower))/(traj_t(upper) - traj_t(lower));
    float y = traj_y(lower) + (traj_y(upper) - traj_y(lower))*(t - traj_t(lower))/(traj_t(upper) - traj_t(lower));
    float theta = traj_theta(lower) + (traj_theta(upper) - traj_theta(lower))*(t - traj_t(lower))/(traj_t(upper) - traj_t(lower));
    Eigen::ArrayXf traj_at_t = Eigen::ArrayXf(4);
    traj_at_t << t,x,y,theta;
    return traj_at_t;
  }
}

Eigen::ArrayXf Trajectory_Tracker::get_error()
{
  tf::StampedTransform transform;
  try{
    listener.lookupTransform("/robot", "/target",  
                             ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  Eigen::ArrayXf error = Eigen::ArrayXf(3);
  tf::Vector3 pose_error;
  tf::Quaternion q = transform.getRotation();
  pose_error = transform.getOrigin();
  error(0) = pose_error.x();
  error(1) = pose_error.y();
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  error(2) = yaw; //This may cause problems if robot is not in plane
  return error;
}

geometry_msgs::Point Trajectory_Tracker::transform_goal_to_robot(float x, float y)
{
  ROS_INFO("WAITING FOR TRANSFORM");
  geometry_msgs::PointStamped p;
  ros::Time now = ros::Time::now();
  if(listener.waitForTransform("/world","/robot",now,ros::Duration(5.0)))
  {
    p.header.stamp = now;
    p.header.frame_id = "/world";
    
    p.point.x = x;
    p.point.y = y;
    p.point.z = 0;
    
    listener.transformPoint("/robot",p,p);
    return p.point;  
  }else{
    ROS_ERROR("FAILED TO GET TRANSFORM BETWEEN /world and /robot");
    return p.point;
  }
  
}

void Trajectory_Tracker::track_trajectory(const planner::gen_trajGoalConstPtr &goal)
{
  ROS_INFO("TRACKING TRAJECTORY");
  float x = goal->x;
  float y = goal->y;

  if(goal->frame == "world"){
    geometry_msgs::Point goal_point = transform_goal_to_robot(x,y);
    x = goal_point.x;
    y = goal_point.y;  
  }
  
  bool following_trajectory = true;
  tf::StampedTransform frame_transform = broadcast_new_traj_frame();
  cout << "generated trajectory" << endl;
  current_trajectory.generate_trajectory(x,y);
  ros::Rate loop_rate(10);
  tf::StampedTransform transform;
  tf::Quaternion q;
  ros::Time start_time = ros::Time::now();
  float t = 0;
  while(t<=(current_trajectory.t_end)+1)
  {
    if (action_server.isPreemptRequested() || !ros::ok())
    {
      float u_v = 0;
      float u_w = 0;
      geometry_msgs::Pose2D cmd_vector;
      cmd_vector.x = u_v;
      cmd_vector.y = 0;
      cmd_vector.theta = u_w;
      cmd_pub.publish(cmd_vector);
      result.success = 0;
      action_server.setSucceeded(result);
      cout << "CANCELING GOAL" << endl;
      return;
    }
    

    t = (ros::Time::now()-start_time).toSec();
    feedback.time = t;
    action_server.publishFeedback(feedback);

    Eigen::ArrayXf traj_at_t = current_trajectory.traj_time_lookup(t);
    
    marker_pub.publish(current_trajectory.points);
    marker_pub.publish(current_trajectory.line_strip);
    marker_pub.publish(current_trajectory.line_list);
    transform.setOrigin( tf::Vector3(traj_at_t(1), traj_at_t(2), 0));

    q.setRPY(0, 0, traj_at_t(3));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/traj_frame", "/target"));

    Eigen::ArrayXf error = get_error();

    float u_v = Kp_x*error(0);
    float u_w = Kp_y*error(1) + Kp_w*error(2);

    // TODO: FEEDFORWARD
    
    geometry_msgs::Pose2D cmd_vector;
    cmd_vector.x = u_v;
    cmd_vector.y = 0;
    cmd_vector.theta = u_w;

    cmd_pub.publish(cmd_vector);

    loop_rate.sleep();
    if(!ros::ok())
    {
      result.success = 0;
      action_server.setSucceeded(result);
      return;
    }
    br.sendTransform(tf::StampedTransform(frame_transform, ros::Time::now(), "/world", "/traj_frame"));
  }
  result.success = 1;
  action_server.setSucceeded(result);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "planner");
  ros::NodeHandle n;


  ros::Rate loop_rate(10);
  Trajectory_Tracker tracker (n);
  // tracker.track_trajectory();
  // loop_rate.sleep();

  // float theta = -3.14;
  // while (ros::ok())
  // {
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }
  ros::spin();

  return 0;
}