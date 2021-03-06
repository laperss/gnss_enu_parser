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

    pose_channel = nh.resolveName("enu");
    pose_pub = nh.advertise<gnss_data::Enu>(pose_channel, 1);

    SetVariableFromParam(nh_gnss_parser, "rate", rate);
    SetVariableFromParam(nh_gnss_parser, "/origin_lat", lat_origin, 57.760981);
    SetVariableFromParam(nh_gnss_parser, "/origin_lon", lon_origin, 16.685680);
    ROS_INFO("GNSS parser origin = (%f, %f)", lat_origin, lon_origin);

    
    int the_message_type = 1;
    /*
    if(nh_gnss_parser.hasParam("msg_type")){
        nh_gnss_parser.getParam("msg_type", the_message_type);
        ROS_INFO("Message type = %d", the_message_type);
    }else{
	ROS_ERROR("No parameter 'msg_type'");
	ros::shutdown();
    }
    */
    if (the_message_type == 1){
        ROS_INFO("Using message type: NMEA");
        msg_type == NMEA;
    }
    else if (the_message_type == 2){
        msg_type == ENU;
        ROS_INFO("Using message type: ENU");
    }
    dt = 1/rate; 

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

    if (msg_type == ENU){
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
    }else{
        ROS_INFO("Parsing NMEA messages");
    }
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

void Parser::Char2MSG(char * input){
    if (msg_type == ENU){
        Char2ENU(input);
    }else{
        Char2NMEA(input);
    }
    
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

void Parser::Char2NMEA(char * input){
    NMEAProtocol nmea;
    char char1[] = "$";
    char chargga[] = "GPGGA";
    char charrmc[] = "GPRMC";
    std::string ggastr = "$GPGGA";
    if (strcmp(input, char1)==0){
            double lon_temp;
            double lat_temp;
            double lon_min_temp;
            double lat_min_temp;
            double lon_sec_temp;
            double lat_sec_temp;


            std::string str(input);

            std::istringstream ss(str);            
            std::string value;
            int count = 0;
            double latitude_tmp;
            double longitude_tmp;
            while(std::getline(ss, value, ',')) {
                if (count == 0){
                    if((value.compare(ggastr)==0)){
                        for (int i=0;i<5;i++) nmea.id[i] = input[1+i];
                    }else{
                        for (int i=0;i<5;i++) nmea.id[i] = charrmc[i];
                        break;
                    }
                }else if (count == 2){
                    //std::cout << "latitude = " << value << '\n';
                    nmea.latitude = stod(value.substr(0,2))  + stod(value.substr(2))/60;
                    
                }else if (count == 4){
                    //std::cout << "longitude = " << value << '\n';
                    nmea.longitude = stod(value.substr(0,3))  + stod(value.substr(3))/60;
                }else if (count == 9){
                    //std::cout << "height = " << value << '\n';
                    nmea.height = stod(value);
                }

              count++;
            }
                
            if (strcmp(chargga, nmea.id)==0){
                    // ADD: Check that values make sense
                //ROS_INFO("latitude: %f", nmea.latitude);
                //ROS_INFO("longitude: %f", nmea.longitude);
                //nmea.latitude = lat_temp + lat_min_temp/60 + lat_sec_temp/3600;
                //nmea.longitude = lon_temp + lon_min_temp/60 + lon_sec_temp/3600;
                
                    double east, north;
                    GPStoEarth(nmea.latitude, nmea.longitude, east, north);
                    gnss_data::Enu enu_msg;
                    enu_msg.east = east;
                    enu_msg.north = north;
                    enu_msg.up = nmea.height;

                    enu_msg.header.seq = seq++;
                    //enu_msg.header.stamp = ros_time_from_week_and_tow(enu.week,enu.tow);
                    enu_msg.header.stamp = ros::Time::now();
                    enu_msg.ratio = nmea.ratio;
                    enu_msg.age = nmea.age;
                    enu_msg.status = nmea.Q;
                    enu_msg.numsat = nmea.ns;
	
                    pose_pub.publish(enu_msg);
                } else{
                    ROS_WARN("NMEA message invalid: %s", input);
            }
        }
        else{
                ROS_WARN("NMEA message invalid: %s", input);        

        }

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
				  double& value, double standard_value){
    if(nh.hasParam(name)){
        nh.getParam(name, value);
    }else{
        value = standard_value;
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
