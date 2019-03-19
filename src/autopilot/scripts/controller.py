#!/usr/bin/env python

import rospy
from kb_utils.msg import Command
from kb_utils.msg import Encoder
from autopilot.msg import Controller_Commands
from autopilot.msg import State # will we need this one?
import numpy as np
import math
import time

class Controller:

    def __init__(self):
        self.path_sub = rospy.Subscriber('controller_commands', Controller_Commands, self.reference_callback, queue_size = 1)
        self.est_sub = rospy.Subscriber('state', State, self.state_callback, queue_size = 1)
        self.command_pub = rospy.Publisher('command', Command, queue_size = 1)

        #Variables for the linear velocity
        self.Kp_v = 0.5
        self.Kd_v = 0.03
        self.Ki_v =0.0# 0.1
        self.prev_error_v = 0.0
        self.integrator_v = 0.0
        self.sigma_v = 2.5
        self.prev_v = 0.0
	self.e_sat_v = 0.3
	self.u_sat_v = 0.3

        self.prev_time = rospy.Time.now()
        self.v_dot = 0.0
        self.v_command = 0.0
        self.v_sat = 1.0

        #Variables for the angular velocity
        self.Kp_psi = 2.0
        self.Kd_psi = 0.0
        self.Ki_psi = 0.0
        self.prev_error_psi = 0.0
        self.integrator_psi = 0.0
        self.prev_psi = 0.0
        self.psi_dot = 0.0
        self.psi_command = 0.0
        self.psi_sat = 1.0
        self.sigma_psi = 1.0
        self.e_sat_psi = .6
        self.u_sat_psi = 1.0

        #Variables for storing values from messages
        self.v_ref = 0.0
        self.psi_ref = 0.0
        self.command_v = 0.0
        self.command_psi = 0.0

    def reference_callback(self, msg):
        #will store the reference value
        print 'Message received'
        self.v_ref = msg.u_c
        self.psi_ref = msg.psi_c #heading angle

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

    def state_callback(self, msg):

        print 'Update state'
        psi = -msg.psi #Heading angle
        v = msg.u   #Body velocity
        now = rospy.Time.now()
        dt = (now - self.prev_time).to_sec()
        self.prev_time = now

        #Angle controller
        error = self.psi_ref - psi
        while error > np.pi:
            error = error - 2 * np.pi

        while error < - np.pi:
            error = error + 2 * np.pi

        if error > self.e_sat_psi:
            error = self.e_sat_psi
        elif error < - self.e_sat_psi:
            error = - self.e_sat_psi

        #Do I need to reset the integrator???

        self.integrator_psi = self.integrator_psi + dt / 2.0 * (error - self.prev_error_psi)
        self.prev_error_psi = error
        self.psi_dot = (2 * self.sigma_psi - dt)/(2 * self.sigma_psi + dt) * self.psi_dot + 2.0 / (2 * self.sigma_psi + dt) * (psi- self.prev_psi)
        self.prev_psi = psi

        u_psi_unsat = self.Kp_psi * error - self.Kd_psi * self.psi_dot + self.Ki_psi * self.integrator_psi

        u_psi = u_psi_unsat

        if u_psi > self.u_sat_psi or u_psi < -self.u_sat_psi:
            self.psi_command = self.u_sat_psi * np.sign(u_psi)
        else:
            self.psi_command = u_psi

        #Anti wind up
        if self.Ki_psi !=0.0:
            self.integrator_psi = self.integrator_psi + dt/self.Ki_psi * (self.psi_command - u_psi)

        #Throttle Controller
        error = self.v_ref - v
    	if error > self.e_sat_v:
    	    error = self.e_sat_v
    	elif error < - self.e_sat_v:
    	    error = - self.e_sat_v

    	if v<0.1 and v>-.1:
    	    self.integrator_v = 0

        self.integrator_v = self.integrator_v + dt / 2.0 * (error - self.prev_error_v)
	    #print self.integrator_v
        self.prev_error_v = error
    	self.v_dot = (2 * self.sigma_v - dt)/(2 * self.sigma_v + dt) * self.v_dot + 2.0 / (2 * self.sigma_v + dt) * (v - self.prev_v)
    	self.prev_v = v

    	u_unsat = self.Kp_v * error - self.Kd_v * self.v_dot + self.Ki_v * self.integrator_v

    	u = u_unsat
        print(u)

    	if u > self.u_sat_v or u < -self.u_sat_v:
    	    self.v_command = self.u_sat_v * np.sign(u)
    	else:
    	    self.v_command = u

    	#Anti wind up. Apply else where also
    	if self.Ki_v != 0.0:
    	    self.integrator_v = self.integrator_v + dt/self.Ki_v * (self.v_command - u)

        vel = Command()
        vel.steer = self.psi_command
        vel.throttle = self.v_command

        self.command_pub.publish(vel)

def main():
    """driver to interface to the Teensy on the KB_Car
    Command Line Arguments
    port -- the serial port to talk to the Teensy
    """
   # arg_fmt = argparse.RawDescriptionHelpFormatter
   # parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
   # parser.add_argument('port', type=str, default='')
   # args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("v_controller", log_level=rospy.DEBUG)

    controller = Controller()
    controller.run()

    print("Done.")

if __name__ == '__main__':
    main()
