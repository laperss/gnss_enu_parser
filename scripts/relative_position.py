#!/usr/bin/env python
from __future__ import print_function
import rospy
import numpy
from gnss_data.msg import Enu


class relative_positioning(object):
    shutdown=False
    def __init__(self):
        enu_str_drone = rospy.resolve_name("/dji12/dji_sdk/enu", caller_id=None)
        enu_str_boat = rospy.resolve_name("/cb90/enu", caller_id=None)
        enu_str = rospy.resolve_name("/relative_position", caller_id=None)
        

        self.uav_sub = rospy.Subscriber(enu_str_drone, Enu, self.uav_callback)
        self.usv_sub = rospy.Subscriber(enu_str_boat, Enu, self.usv_callback)

        self.enu_pub = rospy.Publisher(enu_str, Enu, queue_size=5)
        
        
        self.east_uav = 10000
        self.north_uav = 10000
        self.up_uav = 10000
        
        self.east_usv = 10000
        self.north_usv = 10000
        self.up_usv = 10000
        
    def usv_callback(self, msg):    
        if (abs(msg.east)>500 or abs(msg.north)>1000):
            self.east_usv = msg.east
            self.north_usv = msg.north
            self.up_usv = msg.up
        else:
            rospy.logerr("Received drone position is incorrect: (%f, %f)", msg.east, msg.north)

    def uav_callback(self, msg):    
        if (abs(msg.east)>500 or abs(msg.north)>1000):
            self.east_uav = msg.east
            self.north_uav = msg.north
            self.up_uav = msg.up
        else:
            rospy.logerr("Received drone position is incorrect: (%f, %f)", msg.east, msg.north)
            
    def control_callback(self, msg):
        self.control = msg.axes

    def step(self):        
        if (self.east_uav < 1000 && self.east_usv < 1000)
            enu_msg = Enu()
        
            enu_msg.header.stamp = self.ENUtime
            enu_msg.east = self.east_uav - self.east_usv
            enu_msg.north = self.north_uav - self.north_usv
            enu_msg.up = self.up_uav - self.up_usv
            enu_msg.status = 1
            enu_msg.numsat = 20
            enu_pub.publish(self.enu_msg)
                
                
    def loop(self):
        rate = rospy.Rate(RATE)  # 20hz
        dt = 1.0/RATE
        while not rospy.is_shutdown() and not self.shutdown:
            self.step()
            self.count += 1        
            rate.sleep()
        

    


if __name__ == '__main__':
    rospy.init_node('relative_position')

    try:
        positioner = relative_positioning()
        positioner.loop()
    except rospy.ROSInterruptException:
        rospy.logerr("Could not start the relative position computations")
        pass
