#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from dmp import DMP
from moveit_msgs.msg import DisplayTrajectory
import matplotlib.pyplot as plt
import pickle

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

class Imitate():

    def __init__(self,group):
        rospy.init_node('dmp_fitter',anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(group)

        self.pos_0 = None
        self.pos_1 = None

        self.moving = False
        self.traj = []
        self.dmp_count = 0

    def callback(self,msg):
        if self.js_0 == None:
            self.js_0 = [msg.position.x, msg.position.y, msg.position.z,
            msg.orientation.x, msg.orientation.y, msg.orientation.z,
            msg.orientation.w]
            self.js_1 = [msg.position.x, msg.position.y, msg.position.z,
            msg.orientation.x, msg.orientation.y, msg.orientation.z,
            msg.orientation.w]
        else:
            self.js_0 = self.js_1
            self.js_1 = [msg.position.x, msg.position.y, msg.position.z,
            msg.orientation.x, msg.orientation.y, msg.orientation.z,
            msg.orientation.w]

        if np.sum(np.abs(np.array(self.js_0) - np.array(self.js_1))) > 0.01:
            print('Moving')
            self.traj.append(j_list)
        else:
            if len(self.traj) > 0:

                # Fit DMP to motion segment 
                path = np.array(self.traj)
                dmp = DMP(path[0,:],path[-1,:], Nb=500, dt=0.01,
                d=path.shape[1],jnames=self.joint_names)
                params = dmp.imitate_path(path+1e-5*np.random.randn(path.shape[0],path.shape[1]))
                #print(params)

                dmp.reset_state()
                self.dmp_count += 1

                with open ("./dmp%05d.npy"%self.dmp_count,"w") as f:
                    pickle.dump(dmp,f,pickle.HIGHEST_PROTOCOL)

                y_r,dy_r,ddy_r = dmp.rollout()
                plt.subplot(1,2,1)
                plt.cla()
                plt.plot(y_r)
                plt.subplot(1,2,2)
                plt.cla()
                plt.plot(path)
                plt.draw()
                plt.pause(0.1)
                self.jt.header = msg.header
                self.jt.joint_names = self.joint_names
                jtp = JointTrajectoryPoint()
                for i in range(y_r.shape[0]):
                    jtp.positions = y_r[i,:].tolist()
                    jtp.velocities = dy_r[i,:].tolist()
                    jtp.time_from_start = rospy.Duration(1.0)
                    self.jt.points.append(jtp)
                self.traj_pub.publish(self.jt)

                self.traj = []

            print('Stationary')
            self.jt = JointTrajectory()

    def spin(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            pose = self.group.get_current_pose().pose
            self.callback(r,pose)
            r.sleep()


if __name__ == '__main__':
    plt.ion()
    im = Imitate('l')
    im.spin()
