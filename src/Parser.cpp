 #include <iostream>
#include <stdexcept>
#include <math.h>       /* cos */
#include<vector>
#include<string>
#include<chrono>
#include "Parser.hpp"

static const double EPSILON = 0.00000000001;

ros::Time ros_time_from_week_and_tow(const uint32_t week,
				     const double timeOfWeek) {
    ros::Time rostime(0, 0);
    uint64_t sec = UNIX_TO_GPS_OFFSET + floor(timeOfWeek) + week*7*24*3600;
    uint64_t nsec = (roundf(timeOfWeek * 10) - floor(timeOfWeek)*10)*1e8;
    
    rostime = ros::Time(sec, nsec);
    
    return rostime;
}


Parser::Parser(ros::NodeHandle nh):
    publish(true), seq(0),
    nh_gnss_parser(nh){

    SetVariableFromParam(nh_gnss_parser, "rate", rate);
    dt = 1/rate; 

    pose_channel = nh.resolveName("enu");
    pose_pub = nh.advertise<gnss_data::Enu>(pose_channel, 1);
    reset_service = nh.advertiseService("/filter/reset", &Parser::Reset, this);
}


bool Parser::SetupSerial(Serial * serial){
    ros::Rate loop_rate(5);
    char message[150];
    int bytes = 0;
    int count = 0;
    int error_count = 0;
    ROS_INFO("Setup GNSS parser...");
    ENUProtocol enu;
    double tow = 0;
    while(ros::ok() && count < 10){
	ros::spinOnce();
	bytes = serial->ReadLine(message);
	if (bytes>0)
	    Char2ENUNoSend(message, &enu);

        if (fabs(enu.tow - tow) > 2.0){
            ROS_WARN("resetting time of week = %f", enu.tow);
            tow = enu.tow;
            count = 0;
        }else{
            tow = enu.tow;
            count++;            
        }
        // ADD: Check that values make sense
        if (enu.ns == 0){
            ROS_WARN("ENU message invalid (ns = %i): %s", enu.ns, message);
            count = 0;
            error_count++;
            if (error_count>=10){
                ROS_ERROR("Setup of GNSS parser could not finish: Serial port readings are incorrect.");
                return -1;
                ros::shutdown();            
            }
            ros::Duration(1.0).sleep();
        }else{
            loop_rate.sleep();
        }
    }
    initial_tow = tow;
    ROS_INFO("Setup of GNSS parser finished successfully. ");
    return 0;

}



bool Parser::SetupSocket(Socket * socket){
    ros::Rate loop_rate(5);
    char message[150];
    int bytes = 0;
    int count = 0;
    int error_count = 0;
    ENUProtocol enu;
    double tow = 0;
    while(ros::ok() && count < 10){
	ros::spinOnce();
	bytes = socket->ReadLine(message);
	if (bytes>0)
	    Char2ENUNoSend(message, &enu);

        if (fabs(enu.tow - tow) > 2.0){
            tow = enu.tow;
            count = 0;
        }else{
            tow = enu.tow;
            count++;            
        }
        // ADD: Check that values make sense
        if (enu.ns == 0){
            ROS_WARN("ENU message invalid (ns = %i): %s", enu.ns, message);
            error_count++;
            if (error_count>10){
                ros::shutdown();            
                return -1;
            }
        }         
	loop_rate.sleep();
    }
    initial_tow = tow;
    ROS_INFO("Setup of GNSS parser finished: TOW = %f", initial_tow);
    return 0;

}

bool Parser::Reset(std_srvs::Empty::Request  &req,
		   std_srvs::Empty::Response  &res){
    ROS_INFO("Reset state estimation filter");
    ros::spinOnce();
    
    return 1;
}

void Parser::Char2ENUNoSend(char * input, ENUProtocol * enu){
    sscanf (input,"%d %f %14f %14f %14f %3d %3d %f %f %f %f %f %f %f %f",
	    &enu->week, &enu->tow, &enu->east, &enu->north, &enu->up,
	    &enu->Q, &enu->ns, &enu->sde, &enu->sdn, &enu->sdu,
	    &enu->sden, &enu->sdnu, &enu->sdue, &enu->age, &enu->ratio);

}



void Parser::Char2ENU(char * input){
    ENUProtocol enu;

    sscanf (input,"%d %f %14f %14f %14f %3d %3d %f %f %f %f %f %f %f %f",
	    &enu.week, &enu.tow, &enu.east, &enu.north, &enu.up,
	    &enu.Q, &enu.ns, &enu.sde, &enu.sdn, &enu.sdu,
	    &enu.sden, &enu.sdnu, &enu.sdue, &enu.age, &enu.ratio);

    // ADD: Check that values make sense
    if (enu.ns == 0){
        ROS_WARN("ENU message invalid (ns = %i): %s", enu.ns, input);        
        return;
    }else if (fabs(enu.tow-initial_tow) > 43200){
        ROS_WARN("ENU message invalid (tow = %f): %s", enu.tow, input);        
        return;
    }

    gnss_data::Enu enu_msg;
    enu_msg.east = enu.east;
    enu_msg.north = enu.north;
    enu_msg.up = enu.up;

    enu_msg.header.seq = seq++;
    enu_msg.header.stamp = ros_time_from_week_and_tow(enu.week,enu.tow);

    enu_msg.covariance[0] = enu.sde;
    enu_msg.covariance[1] = enu.sden;
    enu_msg.covariance[2] = enu.sdue;
    enu_msg.covariance[3+0] = enu.sden;
    enu_msg.covariance[3+1] = enu.sdn;
    enu_msg.covariance[3+2] = enu.sdnu;
    enu_msg.covariance[6+0] = enu.sdue;
    enu_msg.covariance[6+1] = enu.sdnu;
    enu_msg.covariance[6+2] = enu.sdu;
    
    enu_msg.ratio = enu.ratio;
    enu_msg.age = enu.age;
    enu_msg.status = enu.Q;
    enu_msg.numsat = enu.ns;
	
    pose_pub.publish(enu_msg);

}


void Parser::SetVariableFromParam(ros::NodeHandle nh,
				  std::string name,
				  int& value){
    if(nh.hasParam(name)){
        nh.getParam(name, value);
    }else{
	ROS_ERROR("No parameter %s", name.c_str());
	ros::shutdown();
    }
}
