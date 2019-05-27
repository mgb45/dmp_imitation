#!/usr/bin/env python
import pickle
from dmp import DMP
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import glob
from matplotlib import pyplot as plt
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import copy

class PlayBack():

    def __init__(self):
        rospy.init_node('dmp_playback',anonymous=True)
        self.traj_pub = rospy.Publisher('motion_segment',JointTrajectory,queue_size=1)
        rospy.Subscriber('joint_states',JointState,self.callback)
        self.header = None
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander(group)

    def load_dmp(self,i):
        with open('dmp%05d.npy'%i,"r") as f:
            obj = pickle.load(f)
        return obj

    def callback(self,msg):
        self.header = msg.header

    def spin(self):
        rate = rospy.Rate(0.5)
        dmp_count = 1
        Ndmp = len(glob.glob('./*.npy'))
        while not rospy.is_shutdown():
            if not (self.header == None):
                dmp = self.load_dmp(dmp_count)
                y_r,dy_r,ddy_r = dmp.rollout()
                waypoints = []
                for i in range(y_r.shape[0]):
                    pose = geometry_msgs.msg.Pose()
                    pose.position.x = y_r[i,0]
                    pose.position.y = y_r[i,1]
                    pose.position.z = y_r[i,2]
                    pose.orientation.x = y_r[i,3]
                    pose.orientation.y = y_r[i,4]
                    pose.orientation.z = y_r[i,5]
                    pose.orientation.w = y_r[i,6]

                    waypoints.append(copy.deepcopy(pose))

		#plt.subplot(1,2,1)
                #plt.plot(y_r)
		#plt.subplot(1,2,2)
		#plt.plot(dy_r)
		#plt.show()





                plan,fraction = group.compute_cartesian_path(wapoints,0.01,0.0)

                #group.execute(plan,wait=True)
                #jtp.positions = y_r[i,:].tolist()
                #jtp.velocities = dy_r[i,:].tolist()
                #jtp.time_from_start = rospy.Duration(1)
                #jt.points.append(jtp)

                #self.traj_pub.publish(jt)
                print('Publishing dmp %d'%dmp_count)
                dmp_count += 1

                if dmp_count > Ndmp:
                    break
            rate.sleep()



if __name__ == '__main__':
    pb = PlayBack()
    pb.spin()

