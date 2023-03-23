#!/usr/bin/env python3
import random,argparse,sys
parser = argparse.ArgumentParser()
import rospy
import math
from std_msgs.msg import Float32MultiArray, Float32
from time import sleep
import numpy as np

class Controller:

    def __init__(self,n_):
        print("node initialised publishing force")
        self.pub=rospy.Publisher("/actual_system/force",Float32MultiArray,queue_size=1)
        jj=np.zeros(shape=(n_,1))+2
        self.msg=Float32MultiArray()
        self.msg.data=jj
        self.r=rospy.Rate(2)
        while not rospy.is_shutdown():
            self.pub.publish(self.msg)
            self.r.sleep()
        rospy.spin()


if __name__ == '__main__':
    # parser.add_argument("--nsections",type=str)
    # args = parser.parse_args()
    # print(args.nsections)
    rospy.init_node('Controller_node', anonymous = True)
    # s = Controller(int(args.nsections))
    s = Controller(5)