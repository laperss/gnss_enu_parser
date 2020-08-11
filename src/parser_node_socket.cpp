#include "ros/ros.h"
#include "Parser.hpp"


int main(int argc, char **argv){    
    ros::init(argc, argv, "parser");
    ros::NodeHandle nh_;
    Parser parser(nh_);

    // Read 'rate', specifying output rate (Hz)
    double rate;
    if(nh_.hasParam("rate")){
        nh_.getParam("rate", rate);
    }else{
	ROS_ERROR("No ROS parameter 'rate' provided.");
	ros::shutdown();
    }

    
    std::string socket_addr;
    int socket_port;
    if(nh_.hasParam("socket_ip")){
        nh_.getParam("socket_ip", socket_addr);
    }else{
	ROS_ERROR("No ROS parameter 'socket_ip' provided.");
	ros::shutdown();
    }
    if(nh_.hasParam("socket_port")){
        nh_.getParam("socket_port", socket_port);
    }else{
	ROS_ERROR("No ROS parameter 'socket_port' provided.");
	ros::shutdown();
    }

    ROS_INFO("Setup the socket connection");
    Socket socket(socket_port, socket_addr);
    socket.SetIP(socket_addr);
    socket.SetPort(socket_port);
    socket.Setup();

    ROS_INFO("Collect data to TOW");    
    parser.SetupSocket(&socket);

    ros::Rate loop_rate(rate);
    char message[150];
    char * ptr = NULL;
    char c;

    int bytes = 0;
    
    while(ros::ok()){
	ros::spinOnce();
	bytes = socket.ReadLine(message);
	if (bytes>0)
	    parser.Char2ENU(message);

	loop_rate.sleep();
    }
    

    parser.Loop();
    return 0;
}
