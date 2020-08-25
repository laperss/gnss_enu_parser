#include "Parser.hpp"
#include <stdexcept>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <fcntl.h> 

Serial::Serial(std::string device)
{   
    this->dev = device;
}
 
Serial::~Serial(){
    close(this->serial_fd);     
}

void Serial::Setup() {
   // Open port
  ROS_INFO("Open port: %s", dev.c_str());
    serial_fd = open(dev.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd < 0) {
      ROS_ERROR("Failed to open port: %i", serial_fd);
      ros::shutdown();	
    }

    // Config
    struct termios config;
    tcgetattr(serial_fd, &config);

    // Set baudrate to 115200
    cfsetispeed(&config, B115200);
    cfsetospeed(&config, B115200);

    config.c_cflag &= ~PARENB;           // no parity
    config.c_cflag &= ~CSTOPB;           // 1 stop bit
    config.c_cflag &= ~CSIZE;  
    config.c_cflag |=  CS8;              // 8 bits
    
    config.c_cflag &= ~CRTSCTS;                       // no flow control
    config.c_cflag |= CREAD | CLOCAL;                 // enable receiver
    config.c_iflag &= ~(IXON | IXOFF | IXANY);        // disable XON/XOFF flow control
    config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 

    // Minimum number of characters for non cannoincal read
    config.c_cc[VMIN]  = 1;
    // Timeout in deciseconds for noncanonical read
    config.c_cc[VTIME] = 0; 

    // Save config
    if (tcsetattr(serial_fd, TCSANOW, &config) < 0) {
        close(serial_fd);
        throw std::runtime_error("Failed to configure port!");
    }

    // Flush RX Buffer
    if (tcflush(serial_fd, TCIFLUSH) < 0) {
        close(serial_fd);
        throw std::runtime_error("Failed to flush buffer!");
    }

    ROS_INFO("Set up serial connection to device %s \n", this->dev.c_str());
}

int Serial::Available() {
    int bytes = 0;
    if (ioctl(serial_fd, TIOCINQ, &bytes) < 0) {
        close(serial_fd);
        throw std::runtime_error("Failed to check buffer!");
    }
    return bytes;
}

void Serial::Read(char * buffer, int amountOfBytes) {
    if (read(serial_fd, buffer, amountOfBytes) < 0) {
        close(serial_fd);
        throw std::runtime_error("Failed to read bytes!");
    }
}

void Serial::ReadByte(char * bytePtr) {
    return Serial::Read(bytePtr, 1);
}

int Serial::ReadLine(char * bytePtr) {
    char c;
    char * ptr = NULL;
    int bytes = 0;
    while (1) {
        if (Available() > 0) {
	    if (read(serial_fd, &c, 1) < 0) {
		close(serial_fd);
		ROS_ERROR("Failed to read bytes!");
		return -1;
	    }
	    if(c ==  '\n'){
		if (ptr != NULL)
		    //*ptr = '\0';
                    ;
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
    
    }
}

