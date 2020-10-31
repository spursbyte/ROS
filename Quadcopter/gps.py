#!/usr/bin/env python




#error values
#----------------
#


self.sample_time=0.060
rospy.Publisher('/drone_command',edrone_cmd,queue_size=1)
rospy.Subscriber('/edrone/gps',NavSatFix,self.gps_data)


def gps_data(self, msg):
    self.lati=msg.latitude
    self.longi=msg.longitude
    self.alti=msg.altitude



def pid(self)
    self.lati
	