 #include <iostream>
#include <stdexcept>
#include <math.h>       /* cos */
#include<vector>
#include<string>
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

    /*
    if (read_from_serial){
	std::string serial_path;
	SetVariableFromParam(nh_gnss_parser, "serial", serial_path);
	serial.Setup();
    } else{
	std::string socket_addr;
	int socket_port;
	SetVariableFromParam(nh_gnss_parser, "socket_ip", socket_addr);
	SetVariableFromParam(nh_gnss_parser, "socket_port", socket_port);
	socket.SetIP(socket_addr);
	socket.SetPort(socket_port);
	socket.Setup();
    }
    */
    ROS_INFO("Setup of GNSS parser finished");

    
}

bool Parser::Reset(std_srvs::Empty::Request  &req,
		   std_srvs::Empty::Response  &res){
    ROS_INFO("Reset state estimation filter");
    ros::spinOnce();
    
    return 1;
}


void Parser::Char2ENU(char * input){
    ENUProtocol enu;

    sscanf (input,"%d %f %14f %14f %14f %3d %3d %f %f %f %f %f %f %f %f",
	    &enu.week, &enu.tow, &enu.east, &enu.north, &enu.up,
	    &enu.Q, &enu.ns, &enu.sde, &enu.sdn, &enu.sdu,
	    &enu.sden, &enu.sdnu, &enu.sdue, &enu.age, &enu.ratio);

    // ADD: Check that values make sense

    gnss_data::Enu enu_msg;
    //pose_msg.header.seq = seq++;
    //pose_msg.header.stamp = ros_time_from_week_and_tow(enu.week,enu.tow);

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


void Parser::Loop(){
    
    ros::Rate loop_rate(rate);
    ros::Duration(1.0).sleep();

    /*
    char message[150];
    char * ptr = NULL;
    char c;

    int bytes;
    while(ros::ok()){
	printf("Loop\n");
	ros::spinOnce();

	if (read_from_serial){
	    bytes = serial.ReadLine(message);
	} else{
	    bytes = socket.ReadLine(message);
	}
	if (bytes>0)
	    Char2ENU(message, enu_msg);

	loop_rate.sleep();
    }
    */
}





//--------------------------- CONVERSIONS ------------------------------ //
double deg2rad (const double degree) { return (degree * M_PI / 180); };
double rad2deg (const double radian) { return (radian * 180 / M_PI); };


void Parser::GPStoEarth(const double lat, const double lon, double& east, double& north){

    if ((std::abs(lon - lon_origin) < EPSILON) &&  (std::abs(lat - lat_origin) < EPSILON)){
	east = 0.0;
	north = 0.0;
    }else{
    
	double lat1r, lon1r, lat2r, lon2r, u, v, x, y;
	// From (origin)
	lon1r = deg2rad(lon_origin);
	lat1r = deg2rad(lat_origin);

	// To (current position)
	lat2r = deg2rad(lat);
	lon2r = deg2rad(lon);

	const double dlon = deg2rad(lon - lon_origin);
	const double dlat = deg2rad(lat - lat_origin);
    
	y = sin(dlon)*cos(lat2r);
	x = cos(lat1r)*sin(lat2r) - sin(lat1r)*cos(lat2r)*cos(dlon);
	u = sin((lat1r - lat2r)/2);
	v = sin((lon1r - lon2r)/2);
	double degree = atan2(y, x); // Degrees from north?
	double distance =  2.0 * 6378138.12 * asin(sqrt(u*u + cos(lat1r) * cos(lat2r)*v*v));

	east = sin(degree)*distance;  // East
	north = cos(degree)*distance;  // North
    }
}


void Parser::SetVariableFromParam(ros::NodeHandle nh,
				  std::string name,
				  std::string& value){
    if(nh.hasParam(name)){
        nh.getParam(name, value);
    }else{
	ROS_ERROR("No parameter %s", name.c_str());
	ros::shutdown();
    }
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
void Parser::SetVariableFromParam(ros::NodeHandle nh,
				  std::string name,
				  double& value){
    if(nh.hasParam(name)){
        nh.getParam(name, value);
    }else{
	ROS_ERROR("No parameter %s", name.c_str());
	ros::shutdown();
    }
}
void Parser::SetVariableFromParam(ros::NodeHandle nh,
				  std::string name,
				  std::vector<double>& value){
    if(nh.hasParam(name)){
        nh.getParam(name, value);
    }else{
	ROS_ERROR("No parameter %s", name.c_str());
	ros::shutdown();
    }
}
