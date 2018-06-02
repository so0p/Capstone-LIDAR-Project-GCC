#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <servo_lidar_test/controller.h>

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>

#include <cmath>

using namespace std;


vector<double> x;
vector<double> y;
vector<double> z;
int size = 0;


void controller_Callback(const servo_lidar_test::controller::ConstPtr& msg)
{

  
//ROS_INFO("got packet: [%s]", msg->x[1]);
  
  size = msg->x.size();

  
 
  for(int i; i<= size; i++)
  {
  
    x.push_back(msg->x[i]);
    cout<< x[200];

    y.push_back(msg->y[i]);
    z.push_back(msg->z[i]);
  }

}

void controller_Callback_two(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //cout<<"HELLO\n";
  //cout<<  msg->ranges[1]<<endl;
}

 void scanner_callBack(const sensor_msgs::LaserScan::ConstPtr& msg)
 {
  cout<<"HELLO\n";
 cout<<  msg->ranges[1]<<endl;
 }


int main( int argc, char** argv )
{

  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Subscriber marker_sub = n.subscribe("controller", 1000, controller_Callback);
 // ros::Subscriber marker_sub_two = n.subscribe("scan", 1000, controller_Callback_two);
  //ros::Subscriber scn = n.subscribe("scan", 100, scanner_callBack);
  ros::spin();


  ros::Rate r(30);


  /*int const ARRAY_SIZE = 500
  float x = 0;*/
 

    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/my_frame";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    geometry_msgs::Point p;

    float f = 0.0;

  while (ros::ok())
  {


    points.points.clear();
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    
    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;


    // Create the vertices for the points and lines

    for (uint32_t i = 0; i < size; ++i)
    {


      /*float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
      float z = 5 * cos(f + i / 100.0f * 2 * M_PI);*/

      
      p.x = x[i];
      p.y = y[i];
      p.z = z[i];
      points.points.push_back(p);
      
      //line_strip.points.push_back(p);

      // The line list needs two points for each line
      //line_list.points.push_back(p);
   
      //line_list.points.push_back(p);
    }

  

    marker_pub.publish(points);
    

    r.sleep();

    
  }



}