# GNSS ENU parser
This script parses ENU (east north up) string data published to either a serial port or a TCP socket. 
Default setting is serial port. The parsed data is published in ROS using the custom message ENU, found in the msgs folder.


The parsed string is in the ENU format of [RTKLIB](http://www.rtklib.com/):

_[week, tow, east, north, up, quality, numsat, sde, sdn, sdu,m sden, sdnu, sdue, age, ratio]_



Note that the following changes have to be made to the RTKRCV settings file to produce a string of this format with TOW:
```
outstr1-format: enu
out-outhead: off
out-timeformat: tow
```
      
To launch the node, run:
```sh
roslaunch gnss_data GNSSconverter.launch
```

To launch the node in socket mode, run:
```sh
roslaunch gnss_data GNSSconverter.launch type:=socket
```

NOTE! The socket mode has some bugs in it and is not recommended
