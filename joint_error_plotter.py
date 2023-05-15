#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

import matplotlib
matplotlib.use('TkAgg')

import matplotlib.pyplot as plt

class JointErrorPlotter:
    def __init__(self):
        self.joint_states = None
        self.joint_cmd = None
        self.joint_pos = [[] for i in range(7)] # create a list of empty lists for each joint position
        self.joint_vel = [[] for i in range(7)] # create a list of empty lists for each joint velocity
        self.joint_eff = [[] for i in range(7)] # create a list of empty lists for each joint effort
        self.joint_cmd_pos = [[] for i in range(7)] # create a list of empty lists for each joint position
        self.joint_state_times = []
        self.joint_cmd_times = []

        rospy.init_node('joint_error_plotter', anonymous=True)
        rospy.Subscriber('/iiwa/joint_states', JointState, self.joint_states_callback)
        rospy.Subscriber('/iiwa/joint_cmd', JointState, self.joint_cmd_callback)
        
        self.rate = rospy.Rate(10) # 10Hz
        self.fig, self.axs = plt.subplots(7, 1, sharex=True, figsize=(8, 10))
        self.run()

    def joint_states_callback(self, msg):
        self.joint_states = msg

    def joint_cmd_callback(self, msg):
        self.joint_cmd = msg

    def run(self):
        joint_state_time = None
        joint_cmd_time = None

        fig, axs = plt.subplots(7, 1, sharex=True)
        lines_joint_pos = []
        lines_joint_vel = []
        lines_joint_eff = []
        lines_joint_cmd = []
        lines_error = []
        for i in range(7):
            line_joint_pos, = axs[i].plot([], [], label='Joint Pos')
            # line_joint_vel, = axs[i].plot([], [], label='Joint Vel')
            # line_joint_eff, = axs[i].plot([], [], label='Joint Eff')
            line_joint_cmd, = axs[i].plot([], [], label='Joint Cmd')
            line_error, = axs[i].plot([], [], label='Error')
            lines_joint_pos.append(line_joint_pos)
            lines_joint_vel.append(line_joint_vel)
            lines_joint_eff.append(line_joint_eff)
            lines_joint_cmd.append(line_joint_cmd)
            lines_error.append(line_error)
            axs[i].set_ylabel('J{}'.format(i+1))
            axs[i].legend("upper left")

        plt.xlabel('Time (s)')

        fig.show()

        while not rospy.is_shutdown():
            if self.joint_states:
                joint_state_time = self.joint_states.header.stamp.to_sec()

                self.joint_state_times.append(joint_state_time)
                for i in range(7):
                    self.joint_pos[i].append(self.joint_states.position[i])
                    self.joint_vel[i].append(self.joint_states.velocity[i])
                    self.joint_eff[i].append(self.joint_states.effort[i])


            if self.joint_cmd:
                joint_cmd_pos = self.joint_cmd.position
                joint_cmd_time = self.joint_cmd.header.stamp.to_sec()

                self.joint_cmd_times.append(joint_cmd_time)
                for i in range(7):
                    self.joint_cmd_pos[i].append(joint_cmd_pos[i])

                self.joint_cmd = None

            for i in range(7):
                error = []
                if self.joint_cmd_times:
                    for pos_time, pos_value in zip(self.joint_state_times, self.joint_pos[i]):
                        closest_time, closest_value = min(zip(self.joint_cmd_times, self.joint_cmd_pos[i]), key=lambda x: abs(x[0] - pos_time))
                        error.append(pos_value - closest_value)
                else:
                    for pos_time, pos_value in zip(self.joint_state_times, self.joint_pos[i]):
                        error.append(0)

                
                lines_joint_pos[i].set_data(self.joint_state_times, self.joint_pos[i])
                # lines_joint_vel[i].set_data(self.joint_state_times, self.joint_vel[i])
                # lines_joint_eff[i].set_data(self.joint_state_times, self.joint_eff[i])
                lines_joint_cmd[i].set_data(self.joint_cmd_times, self.joint_cmd_pos[i])
                lines_error[i].set_data(self.joint_state_times, error)
                axs[i].relim()
                axs[i].autoscale_view()
                axs[i].legend("upper left")

            #print(f"Last times: state{joint_state_time} cmd{joint_cmd_time} now{rospy.Time.now().to_sec()}")

            plt.xlabel('Time')
            plt.suptitle('Joint Position Plot')
            fig.canvas.draw()
            fig.canvas.flush_events()

            
            if self.joint_cmd:
                error = [0 for i in range(7)]
                for i in range(7):

                        for cmd_time, cmd_value in zip(self.joint_cmd_times, self.joint_cmd_pos[i]):
                            closest_time, closest_value = min(zip(self.joint_state_times, self.joint_pos[i]), key=lambda x: abs(x[0] - cmd_time))
                            error[i] += abs(cmd_value - closest_value)
                        
                print("Joint errors: ",[(i, error[i]) for i in range(7)], " Total error: ",sum(error))
                    
            self.rate.sleep()

if __name__ == '__main__':
    try:
        joint_error_plotter = JointErrorPlotter()
    except rospy.ROSInterruptException:
        pass