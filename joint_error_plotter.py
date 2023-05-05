#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import matplotlib.pyplot as plt

class JointErrorPlotter:
    def __init__(self):
        self.joint_states = None
        self.joint_cmd = None
        self.error = []
        self.time = []

        rospy.init_node('joint_error_plotter', anonymous=True)
        rospy.Subscriber('/iiwa/joint_states', JointState, self.joint_states_callback)
        rospy.Subscriber('/iiwa/joint_cmd', JointTrajectory, self.joint_cmd_callback)
        
        self.rate = rospy.Rate(10) # 10Hz
        self.run()

    def joint_states_callback(self, msg):
        self.joint_states = msg

    def joint_cmd_callback(self, msg):
        self.joint_cmd = msg

    def run(self):
        while not rospy.is_shutdown():
            if self.joint_states and self.joint_cmd:
                current_time = rospy.Time.now().to_sec()
                self.time.append(current_time)

                joint_cmd_pos = self.joint_cmd.points[-1].positions
                joint_state_pos = self.joint_states.position

                error = np.array(joint_cmd_pos) - np.array(joint_state_pos)
                self.error.append(error)

                plt.plot(self.time, self.error)
                plt.xlabel('Time (s)')
                plt.ylabel('Joint Error (rad)')
                plt.title('Joint Error Plot')
                plt.draw()
                plt.pause(0.0001)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        joint_error_plotter = JointErrorPlotter()
    except rospy.ROSInterruptException:
        pass
