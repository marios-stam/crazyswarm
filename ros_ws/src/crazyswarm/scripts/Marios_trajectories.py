#!/usr/bin/env python3

import rospy
from rospy.client import INFO
from tf import TransformListener
import numpy as np

from pycrazyswarm.crazyflie import Crazyflie
from uav_trajectory import Trajectory
from crazyswarm.msg import TrajectoryPolynomialPieceMarios

import os
import uav_trajectory


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


def callback(piece_pol: TrajectoryPolynomialPieceMarios):
    print("Callback")
    cfid = piece_pol.cf_id
    print("cfid:", cfid)

    lines = int(len(piece_pol.poly_x)/8)

    print(len(piece_pol.poly_x))
    # input()
    x = np.array(piece_pol.poly_x).reshape((lines, 8))
    y = np.array(piece_pol.poly_y).reshape((lines, 8))
    z = np.array(piece_pol.poly_z).reshape((lines, 8))
    yaw = np.array(piece_pol.poly_yaw).reshape((lines, 8))
    durations = np.array(piece_pol.durations).reshape((lines, 1))

    print("x:", x.shape)
    print("y:", y.shape)
    print("z:", z.shape)
    print("yaw:", yaw.shape)
    print("durations:", durations.shape)

    matrix = np.zeros((lines, 1+8*4))  # 8 coeffs per x,y,z,yaw + 1 for duration
    matrix[:, 0] = durations.flatten()
    matrix[:, 1:9] = x
    matrix[:, 9:17] = y
    matrix[:, 17:25] = z
    matrix[:, 25:33] = yaw

    file_name = "piecewise_pole.csv"
    np.savetxt(file_name, matrix, delimiter=",")

    traj = uav_trajectory.Trajectory()
    traj.loadcsv(file_name)  # TODO:Loaf trajectory without using file


if __name__ == "__main__":

    rospy.init_node("CrazyflieDistributed")

    # cf = None
    # for crazyflie in rospy.get_param("crazyflies"):
    #     cfid = int(crazyflie["id"])
    #     print(cfid)
    #     if cfid == int(crazyflie["id"]):
    #         initialPosition = crazyflie["initialPosition"]
    #         tf = TransformListener()
    #         cf = Crazyflie(cfid, initialPosition, tf)
    #         print("Found cf with id:", cfid)
    #         break

    rospy.Subscriber('piece_pol', TrajectoryPolynomialPieceMarios, callback)

    rospy.spin()

    # if cf is None:
    #     rospy.logwarn("No CF with required ID {} found!".format(cfid))
    # else:
    #     pass
    #     run(tf, cf)
    # # pos = cf.initialPosition.copy()
    # cf.takeoff(targetHeight=0.5, duration=2.0)
