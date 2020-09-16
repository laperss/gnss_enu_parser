#include "ros/ros.h"
#include "Parser.hpp"


int main(int argc, char **argv){    
    ros::init(argc, argv, "parser");
    ros::NodeHandle nh_;
    Parser parser(nh_);

    int result;
        
    // Read 'rate', specifying output rate (Hz)
    double rate;
    if(nh_.hasParam("rate")){
        nh_.getParam("rate", rate);
    }else{
	ROS_ERROR("No ROS parameter 'rate' provided.");
	ros::shutdown();
    }


    std::string serial_path;
    if(nh_.hasParam("serial")){
        nh_.getParam("serial", serial_path);
    }else{
	ROS_ERROR("No ROS parameter 'serial' provided.");
	ros::shutdown();
    }
    Serial serial(serial_path);
    serial.Setup();

    
    result = parser.SetupSerial(&serial);

    if (result == 0){
        ros::Rate loop_rate(rate);
        char message[250];
        int bytes = 0;
        
        while(ros::ok()){
            ros::spinOnce();
            bytes = serial.ReadLine(message);

            
            if (bytes>0){
                parser.Char2MSG(message);
            }
            loop_rate.sleep();
        }
    }
    return 0;
}
