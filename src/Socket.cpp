#include "Parser.hpp"
#include <stdexcept>
#include <string.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

Socket::Socket(const int port,  std::string ip_addr){   
    // Open port
    this->ip_addr = ip_addr;
    this->port = port;


}

Socket::~Socket(){
    close(this->socket_fd);
}


int Socket::Setup(){
    // Create socket for receiving flightgear data stream. 
    if ((this->socket_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0)  { 
        ROS_ERROR("Socket creation error"); 
        return -1; 
    }     
    ROS_INFO("CREATE SOCKET STREAM %i TO PORT %i\n", this->socket_fd, this->port);

    int enable = 1;
    if (setsockopt(this->socket_fd, SOL_SOCKET, SO_REUSEADDR,
		   &enable, sizeof(int)) < 0)
	ROS_ERROR("setsockopt(SO_REUSEADDR) failed");

    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(this->socket_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

    serv_addr.sin_family = AF_INET; 
    serv_addr.sin_port = htons(this->port); 
       
    // Convert IPv4 and IPv6 addresses from text to binary form 
    if(inet_pton(AF_INET, "127.0.0.1", &serv_addr.sin_addr)<=0)  
    { 
        ROS_ERROR("\nInvalid address/ Address not supported "); 
        return -1; 
    } 
   
    if (connect(this->socket_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0){ 
        ROS_ERROR("\nConnection Failed "); 
        return -1; 
    }     
    

    ROS_INFO("Set up TCP connection to read port %i \n", this->port);
}

void Socket::Reconnect(){
    if (connect(this->socket_fd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0){ 
        ROS_ERROR("Connection Failed "); 
    }else{
        ROS_INFO("Connection OS "); 
    }
    

}

void Socket::SetIP(std::string addr){
    this->ip_addr = addr;
}

void Socket::SetPort(int port){
    this->port = port;
}

int Socket::Read(char * buffer, int len) {
    //socklen_t addrlen = sizeof(this->socket_addr);  
    //int n = recvfrom(this->socket_fd, buffer, len, 0, (struct sockaddr *)&this->socket_addr, &addrlen);

    int n = read(this->socket_fd , buffer, len); 
    return n;
}

int Socket::Available() {
    int bytes = 0;
    if (ioctl(socket_fd, TIOCINQ, &bytes) < 0) {
        close(socket_fd);
        ROS_ERROR("Failed to check buffer!");
    }
    return bytes;
}


int Socket::ReadByte(char * buffer) {
    int n = read(this->socket_fd , buffer, 1);
    if (n==0){
	//Reconnect();
    }
    return n;
}

int Socket::ReadLine(char * bytePtr) {
    char c;
    char * ptr = NULL;
    int bytes = 0;
    bool available = false;
    for (int i=0; i<100; i++){
	if (Available() > 0) {
	    available = true;
	    break;
	}
    }
    if (available) {
	while (Available() > 0) {
	    if (read(this->socket_fd , &c, 1) <= 0) {
		close(fd);
		ROS_ERROR("Failed to read bytes!");
		return -1;
	    }
	    if(c ==  '\n'){
		if (ptr != NULL)
		    *ptr = '\0';
		bytes++;
		return bytes;
	    } else{
		if (ptr != NULL) {
		    *ptr++ = c;
		    bytes++;
		} else {
		    ptr = bytePtr;
		    *ptr++ = c;
		    bytes++;
		}
	    }
		
		
	}
    
    }else{
	if (bytes==0){
	    Reconnect();
	}
    }
    
}


