#!/usr/bin/env python




#error values
#----------------
#

self.lat_prev_values=0.0
self.long_prev_values=0.0
self.alt_prev_values=0.0
self.sample_time=0.060
self.command=rospy.Publisher('/drone_command',edrone_cmd,queue_size=1)
rospy.Subscriber('/edrone/gps',NavSatFix,self.gps_data)
rospy.Subscriber('/input_gps',custom_gps,self.input_gps_callback)
rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)


def gps_data(self, msg):
    self.lat=msg.latitude
    self.long=msg.longitude
    self.alt=msg.altitude



def input_gps_callback(self,msg):
	self.setpoint_lat=msg.set_latitude
	self.setpoint_long=msg.set_longitude
	self.setpoint_alt=msg.set_altitude

def imu_callback(self, msg):

   self.drone_orientation_quaternion[0] = msg.orientation.x
   self.drone_orientation_quaternion[1] = msg.orientation.y
   self.drone_orientation_quaternion[2] = msg.orientation.z
   self.drone_orientation_quaternion[3] = msg.orientation.w

def pid(self):

	(self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) = tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1], self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])
	self.lat_error=self.setpoint_lat-self.lat
	self.long_error=self.setpoint_long-self.long
	self.alt_error=self.setpoint_alt-self.alt


	self.lat_diff_err=abs(self.lat_error)-self.lat_prev_values
	self.long_diff_err=abs(self.long_error)-self.long_prev_values
	self.alt_diff_err=abs(self.alt_error)-self.alt_prev_values

	self.lat_iterm=(self.lat_iterm+self.lat_error)*self.Ki
	self.long_iterm=(self.long_iterm+self.long_error)*self.Ki
	self.alt_iterm=(self.alt_iterm+self.alt_error)*self.Ki

	self.output_lat=self.Kp*self.lat_error+self.lat_iterm+self.Kd*(self.lat_diff_err)
    self.output_long=self.Kp*self.long_error+self.long_iterm+self.Kd*(self.long_diff_err)
    self.output_alt=self.Kp*self.alt_error+self.alt_iterm+self.Kd*(self.alt_diff_err)


    if(self.lat_error<0):
    	self.out_pitch=abs(self.lat_error)*100.0
    	self.command.rcPitch=(self.drone_orientation_euler[1]-self.out_pitch)
    	self.command.publish(self.pitch_out)
    if(self.lat_error>=0):
        self.out_pitch=abs(self.lat_error)*100.0
        self.command.rcPitch=(self.drone_orientation_euler[1]+self.out_pitch)
    if(self.long_error<0):
    	self.out_yaw=abs(self.long_error)*100.0
    	self.command.rcYaw=(self.drone_orientation_euler[2]-self.out_yaw)
    if(self.long_error>=0):
    	self.out_yaw=abs(self.long_error)*100.0
        self.out_yaw=abs(self.long_error)*100.0
    	self.command.rcYaw=(self.drone_orientation_euler[2]+self.out_yaw)




