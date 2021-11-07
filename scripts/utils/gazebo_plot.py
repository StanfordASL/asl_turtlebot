#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
import numpy as np
from gazebo_msgs.msg import ModelStates
from threading import Lock

BUF_SIZE = 200
DIV = 250

class GazeboPlot:

    def __init__(self):
        self.lock = Lock()

        self.x = []
        self.y = []
        self.t = np.linspace(-1,0,BUF_SIZE)
        self.v = [0] * BUF_SIZE
        self.count = 0

        plt.ion()
        self.fig = plt.figure()
        self.pos_ax = self.fig.add_subplot(211)
        self.pos_ax.set_xlim(-5,5)
        self.pos_ax.set_ylim(-5,5)
        self.pos_ax.set_xlabel('x position')
        self.pos_ax.set_ylabel('y position')
        self.pos_ax.grid("on")

        self.path, = self.pos_ax.plot(self.x, self.y, 'h')
        self.speed_ax = self.fig.add_subplot(212)
        self.speed_ax.set_xlim(-1,0)
        self.speed_ax.set_ylim(-.3,.3)
        self.speed_ax.set_ylabel('velocity')
        self.speed_ax.grid("on")
        self.speed, = self.speed_ax.plot(self.t, self.v)

        plt.tight_layout()

    def gazebo_callback(self, msg):
        if self.count == 0:
            if "turtlebot3_burger" in msg.name:
                pose = msg.pose[msg.name.index("turtlebot3_burger")]
                twist = msg.twist[msg.name.index("turtlebot3_burger")]
                self.lock.acquire()
                if len(self.x)==0 or len(self.y)==0:
                    self.x = [pose.position.x] * BUF_SIZE
                    self.y = [pose.position.y] * BUF_SIZE
                else:
                    self.x = self.x[1:]
                    self.y = self.y[1:]
                    self.x.append(pose.position.x)
                    self.y.append(pose.position.y)
                self.v = self.v[1:]
                self.v.append(np.sqrt(twist.linear.x**2+twist.linear.y**2))
                self.lock.release()
        if self.count==DIV:
            self.count = 0
        else:
            self.count += 1

    def run(self):
        rospy.init_node('gazebo_plot', anonymous=True)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.lock.acquire()
            self.path.set_xdata(self.x)
            self.path.set_ydata(self.y)
            self.speed.set_ydata(self.v)
            self.lock.release()
            #self.fig.canvas.draw()
            #plt.pause(1e-2)
            self.fig.canvas.draw_idle()
            self.fig.canvas.start_event_loop(1e-2)
            rate.sleep()

if __name__ == '__main__':
    gz_plot = GazeboPlot()
    gz_plot.run()
