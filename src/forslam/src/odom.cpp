#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include<unistd.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<termios.h>
#include<errno.h>
#include<arpa/inet.h>
#include<sys/time.h>
#include<errno.h>
#include<sys/socket.h>

void microsecond_sleep(unsigned long ms)
{
   struct timeval tv;
   tv.tv_sec=ms/1000;
   tv.tv_usec=(ms%1000)*1000;
   int err; 
   do{
     err=select(0,NULL,NULL,NULL,&tv);
    }while(err<0);
}

   double vth=0.0;
void chatterCallback(const sensor_msgs::Imu& msg)
{
//   char bufferSendGyro[]="AA08A30A20.7A";

   vth = msg.angular_velocity.z;	
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;
//   ros::Publisher IMU_pub = n.advertise<sensor_msgs::Imu>("imu", 20); 
	ros::Subscriber sub = n.subscribe("imu", 1, chatterCallback);

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  int sock=socket(AF_INET,SOCK_STREAM,0);
   struct sockaddr_in serv_addr;
   memset(&serv_addr,0,sizeof(serv_addr));
   serv_addr.sin_family=AF_INET;
   serv_addr.sin_addr.s_addr=inet_addr("192.168.1.200");
   serv_addr.sin_port=htons(4999);
   int connectStatus = connect(sock,(struct sockaddr*)&serv_addr,sizeof(serv_addr)) ;

   ROS_INFO("CONNECT status:\t %d",connectStatus);
   ////////////////////////////////////////////////////////////
  char bufferSendGyro[]="AA10AA";
  char bufferRecvGyro[128];

  double* gyroStatus =(double*)bufferRecvGyro;
  memset(bufferRecvGyro,0,sizeof(bufferRecvGyro));
///////////////////////////////////////////////////////////

   double vx=0.0;
   double vy=0.0;
//    double vth=0.0;
   double th1=0.0;
   double th2=0.0;
   double th11=0.0;
   double th21=0.0;
   double b=0.360;

  ros::Rate r(10.0);

    
  while(1){
  //  microsecond_sleep(50);
    ros::spinOnce();               // check for incoming messages



    send(sock,bufferSendGyro,sizeof(bufferSendGyro),0);
    recv(sock,bufferRecvGyro,sizeof(bufferRecvGyro),0);
    current_time = ros::Time::now();
    double* gyroStatus =(double*)bufferRecvGyro;

    th1=gyroStatus[11]-th11;
    th2=gyroStatus[9]-th21;
    th11=gyroStatus[11];
    th21=gyroStatus[9];

    // ROS_INFO("th\t%f\th21\t%f",th11,th21);

     double dt = (current_time - last_time).toSec();
     vx=(th2+th1)/2.0*0.13/180*3.14/dt;

    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    //r.sleep();
  }
}