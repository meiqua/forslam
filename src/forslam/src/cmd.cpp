#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

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
#include<sstream>

int sock=socket(AF_INET,SOCK_STREAM,0);
   struct sockaddr_in serv_addr;
   int connectStatus ;

std::string to_string(int n){
	std::stringstream newstr;
	newstr<<n;
	return newstr.str();
}

void chatterCallback(const geometry_msgs::Twist& msg)
{
//   char bufferSendGyro[]="AA08A30A20.7A";

  double vx = msg.linear.x;
  double vth = -msg.angular.z;

  int vx_ = int(vx*163.4);
  int vth_ = int(vth*285.7);

  std::string bufferSendGyro = "AA08A" + to_string(vx_) + "A" + to_string(vth_) + "AA";

  ROS_INFO("%s\n",bufferSendGyro.c_str());
  send(sock,bufferSendGyro.c_str(),sizeof(bufferSendGyro),0);
	
}

int main(int argc, char *argv[])
{
   memset(&serv_addr,0,sizeof(serv_addr));
      serv_addr.sin_family=AF_INET;
   serv_addr.sin_addr.s_addr=inet_addr("192.168.1.200");
   serv_addr.sin_port=htons(4999);
connectStatus = connect(sock,(struct sockaddr*)&serv_addr,sizeof(serv_addr)) ;
	ros::init(argc, argv, "cmd");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("cmd_vel", 1, chatterCallback);

	ros::spin();

	return 0;
}