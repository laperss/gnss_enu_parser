#include "ros/ros.h"
#include "ros/topic.h"
#include "tf/transform_datatypes.h"
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/String.h"
#include <gnss_data/Enu.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include <vector>
#include <thread>


#include <unistd.h> // write(), read(), close()

#include <sys/types.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 

#pragma once

# define GPS_UNIX_OFFSET 315964800
// GPS time started on 6/1/1980 while UNIX time started 1/1/1970 this is the difference between those in seconds
# define LEAP_SECONDS 18
// GPS time does not have leap seconds, UNIX does (as of 1/1/2017 - next one is probably in 2020 sometime unless there is some crazy earthquake or nuclear blast)
# define UNIX_TO_GPS_OFFSET (GPS_UNIX_OFFSET - LEAP_SECONDS)

#define MAXLINE 128 
#define BUFSIZE 256


typedef struct __attribute__((__packed__))  {
    char id[6];
    float UTC = 0.0;
    float latitude = 0.0;
    char  lat_dir;
    float longitude = 0.0;
    char  lon_dir;
    float altitude = 0.0;
    int Q = 0.0;
    int ns = 0.0;
    float hdop = 0.0;
    float height = 0.0;
    char  height_unit;
    float age = 0.0;
    float ratio = 0.0;
    int reference = 0;
} NMEAProtocol;


typedef struct __attribute__((__packed__))  {
    int week = 0.0;
    float tow = 0.0;
    float east = 0.0;
    float north = 0.0;
    float up = 0.0;
    int Q = 0.0;
    int ns = 0.0;
    float sde = 0.0;
    float sdn = 0.0;
    float sdu = 0.0;
    float sden = 0.0;
    float sdnu = 0.0;
    float sdue = 0.0;
    float age = 0.0;
    float ratio = 0.0;
} ENUProtocol;



class Socket {
private:
    int fd;
    struct sockaddr_in socket_addr_in;
    struct sockaddr_in socket_addr_other;
    struct sockaddr* socket_addr;
    struct sockaddr_in serv_addr;
    int socket_addr_size;
    socklen_t socket_addr_size_other;

    std::string ip_addr;
    int port;
    int socket_fd;
    char buffer[MAXLINE];

    void Reconnect();
    
public:
    Socket(const int read_port,  std::string ip_addr);
    ~Socket();
    void SetIP(std::string addr);
    void SetPort(int port);
    int Setup();
    int Read(char * buffer, int amountOfBytes);
    int ReadByte(char * bytePtr);
    int Available();
    int ReadLine(char * bytePtr);
};

class Serial {
private:
    int serial_fd;
public:
    std::string dev;
    Serial(std::string device);
    ~Serial();

    void Setup();
    int Available();
    void Read(char * buffer, int amountOfBytes);
    void ReadByte(char * bytePtr);
    int ReadLine(char * bytePtr);
};


class Parser {
public:
    Parser(ros::NodeHandle nh);
    bool SetupSerial(Serial * serial);
    bool SetupSocket(Socket * socket);
    bool Reset(std_srvs::Empty::Request  &req,
	       std_srvs::Empty::Response &res);
    void Char2MSG(char * input);   
    void Char2ENU(char * input);   
    void Char2NMEA(char * input);   
    void Char2ENUNoSend(char * input, ENUProtocol * enu);   
    //Serial serial;
    //Socket socket;

    // Discrete time step
    double dt;
    double rate;
    int seq;

private:
    static constexpr int n_states = 18;
    static constexpr int n_inputs = 4;
    static constexpr int n_measured = 9;

    static constexpr double gravity = 9.8066;
    double lat_origin;
    double lon_origin;
    double initial_tow;
    // Local coordinates of boat GPS relative to landing platform
    double gps_forward, gps_left;

    enum message_types{
        NMEA    =  1,
        ENU     =  2,
    };

    message_types msg_type; 
    
    ros::NodeHandle nh_gnss_parser;

    void SetVariableFromParam(ros::NodeHandle, std::string name, double& value);
    void SetVariableFromParam(ros::NodeHandle, std::string name,
                              std::string& value);
    void SetVariableFromParam(ros::NodeHandle, std::string name, int& value);
    void SetVariableFromParam(ros::NodeHandle, std::string name, double& value, double standard_value);
    void SetVariableFromParam(ros::NodeHandle, std::string name,
                              std::vector<double>& value);
    
    // Conversions
    void GPStoEarth(const double lat, const double lon, double& east, double&north);
    
    bool publish;
    
   
    // Publishers
    ros::Publisher pose_pub;

    // Servers
    ros::ServiceServer reset_service;
    
    // Channel names
    std::string pose_channel;

    
    
};
