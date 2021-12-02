#!/usr/bin/env python3

import rospy
from rospy.client import INFO
from tf import TransformListener
import numpy as np

from pycrazyswarm.crazyflie import Crazyflie
from uav_trajectory import Trajectory

import os


def run(tf, cf: Crazyflie):
    # Advanced users: Use the following to get state information of neighbors
    # position, quaternion = tf.lookupTransform("/world", "/cf" + str(cfid), rospy.Time(0))
    tr = Trajectory()
    print("Current directory:", os.getcwd())
    tr.loadcsv("/home/marios/crazyswarm/ros_ws/src/crazyswarm/scripts/figure8.csv")

    print("Crazyflies ROS parameter")
    print(rospy.get_param("crazyflies"))

    pos = cf.initialPosition.copy()
    print()
    cf.takeoff(targetHeight=0.5, duration=2.0)
    rospy.sleep(2)

    cf.goTo([0, 0, 0.5], yaw=0, duration=2.0)
    rospy.sleep(2)
    print("hello")
    cf.uploadTrajectory(trajectoryId=0, pieceOffset=0, trajectory=tr)
    print("Uploaded")
    cf.startTrajectory(trajectoryId=0)
    rospy.sleep(tr.duration)

    cf.land(targetHeight=0.02, duration=2.0)
    rospy.sleep(2)

    cf.cmdStop()


if __name__ == "__main__":

    rospy.init_node("CrazyflieDistributed", anonymous=True)
    cf = None
    for crazyflie in rospy.get_param("crazyflies"):
        cfid = int(crazyflie["id"])
        print(cfid)
        if cfid == int(crazyflie["id"]):
            initialPosition = crazyflie["initialPosition"]
            tf = TransformListener()
            cf = Crazyflie(cfid, initialPosition, tf)
            print("Found cf with id:", cfid)
            break

    if cf is None:
        rospy.logwarn("No CF with required ID {} found!".format(cfid))
    else:
        pass
        run(tf, cf)
    # pos = cf.initialPosition.copy()
    # cf.takeoff(targetHeight=0.5, duration=2.0)
