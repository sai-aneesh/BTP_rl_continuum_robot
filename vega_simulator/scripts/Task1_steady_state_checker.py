#!/usr/bin/env python3
import random,argparse,sys
parser = argparse.ArgumentParser()
import rospy
import math
from std_msgs.msg import Float32MultiArray, Float32
from time import sleep
import numpy as np


class steady_state_checker:
    def __init__(self, epsilon):
        self.epsilon = epsilon
        self.states_msg=Float32MultiArray()
        print('steady state checking node initialised')
        # publishers
        self.state_pub=rospy.Publisher("/steady_system/state",Float32MultiArray,queue_size=1) #giving state to controller
        self.complete_state_pub=rospy.Publisher("/steady_system/complete_state",Float32MultiArray,queue_size=1) #giving complete state to controller
        self.force_pub=rospy.Publisher("/actual_system/force",Float32MultiArray,queue_size=1) #giving forces t0 vega
        # subscribers
        rospy.Subscriber("/actual_system/state",Float32MultiArray,self.state_callback,queue_size=1) #getting state from vega continuously
        rospy.Subscriber("/actual_system/complete_state",Float32MultiArray,self.complete_state_callback,queue_size=1) #getting complete state from vega continuously
        rospy.Subscriber("/steady_system/force",Float32MultiArray,self.force_callback,queue_size=1) #getting forces from controller
        self.x_prev = 0
        self.y_prev = 0
        self.count = 0
        self.steady = True
        self.steady2 = False
        self.force_new = Float32MultiArray()
        self.force_previous = Float32MultiArray()
        rospy.spin()
        sleep(2)

    def state_callback(self, msg):
        tip_pos = np.array(msg.data)
        # rospy.loginfo(tip_pos)

        # r=rospy.Rate(1)
        x = tip_pos[0]
        y = tip_pos[1]

        if (np.sqrt((self.x_prev - x)**2 + (self.y_prev - y)**2) < self.epsilon): #counter and repeat this condition 6 times before concluding steady state
            self.count += 1
        else:
            self.count = 0
        
        self.force_pub.publish(self.force_previous)
        # rospy.loginfo("B4.sending the OLD force to vega")
        # r.sleep()

        # update xprev
        self.x_prev = x
        self.y_prev = y
        # rospy.spin()
        # rospy.loginfo("STUCK HERE")
        # rospy.loginfo(self.count)
        if(self.count == 6):
            self.steady = True
            self.steady2 = True
            # self.count = 0                            #uncommenting this line messes up the logic
            self.states_msg.data = np.array([x, y])
            self.state_pub.publish(self.states_msg)
            rospy.loginfo("B5.sending the steady state to RL_controller")


    def complete_state_callback(self, msg):
        c_state = msg
        if(self.steady2):
            rospy.loginfo("B6.publishing steady complete state")
            self.complete_state_pub.publish(c_state)
            self.steady2 = False



    def force_callback(self, msg):
        self.force_new= msg
        # rospy.spin()
        rospy.loginfo("B1. steady state received force from controller")
        if(self.steady):
            self.force_pub.publish(self.force_new)
            rospy.loginfo("B2.sending the new force to vega")
            self.force_previous = self.force_new
            self.steady = False
            self.count = 0

        else:
            self.force_pub.publish(self.force_previous)
            rospy.loginfo("B3.sending the OLD force to vega")




if __name__ == '__main__':
    # parser.add_argument("--nsections",type=str)
    # args = parser.parse_args()
    # print(args.nsections)
    rospy.init_node('steady_state_checker_node', anonymous = True)
    # s = Controller(int(args.nsections))
    s = steady_state_checker(0.005)
