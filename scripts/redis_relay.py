#!/usr/bin/env python

import argparse
import base64
import json
import socket
import yaml

import numpy as np
import redis
import rospy
import tf

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

class RedisRelay:

    def __init__(self, redis_hostname, self_hostname):
        self.redis = redis.Redis(host=redis_hostname)

        rospy.init_node('redis_relay')
        self.sub = rospy.Subscriber('/map', OccupancyGrid, self.map_update_callback, queue_size=1)

        self.hostname = self_hostname

        print("Initializing transform listener")
        self.tf = tf.TransformListener()
        self.tf_pub = tf.TransformBroadcaster()
        print("Initialized transform listener")

        self.time = rospy.Time()
        self.seq = 0
        self.seq_adversaries = {}

    def map_update_callback(self, data):
        """
        Listen for map updates in local ROS and publish to Redis.
        """
        # Extract map data
        arr_map = np.array(data.data, dtype=np.int8)
        arr_map = arr_map.reshape((data.info.height, data.info.width))
        str_map = base64.encodestring(arr_map.tobytes()).decode('ascii')
        data.data = None

        # Convert msg to JSON string
        yaml_pose = yaml.load(str(data))
        str_pose = json.dumps(yaml_pose)

        # Publish map to Redis
        self.redis.set("{}::{}".format(self.hostname, 'map'), str_pose)
        self.redis.set("{}::{}::data".format(self.hostname, 'map'), str_map)

        # import cv2 as cv
        # arr_map[arr_map<0] = 255
        # img_map = arr_map.astype(np.uint8)
        # cv.imwrite('map1.png', img_map)

    def publish_pose_to_redis(self):
        """
        Get local pose from tf and publish it to Redis.
        """
        if not self.tf.frameExists('base_footprint'):
            return

        try:
            t = self.tf.getLatestCommonTime('base_footprint','map')
            if t == self.time:
                return
            pos, quat = self.tf.lookupTransform('map', 'base_footprint', t)
            self.time = t
        except:
            return

        pose = PoseStamped()
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2]
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        pose.header.stamp.secs = t.secs
        pose.header.stamp.nsecs = t.nsecs
        pose.header.seq = self.seq

        yaml_pose = yaml.load(str(pose))
        str_pose = json.dumps(yaml_pose)

        self.redis.set("{}::{}".format(self.hostname, 'tf'), str_pose)

        self.seq += 1

    def get_poses_from_redis(self):
        """
        Get adversary poses from Redis and publish to tf.
        """
        str_adversary_poses = self.redis.get("{}::adversaries::tf".format(self.hostname))
        if str_adversary_poses is None:
            return

        json_adversary_poses = json.loads(str_adversary_poses)
        for adversary in json_adversary_poses:
            json_posestamped = json_adversary_poses[adversary]
            json_pose = json_posestamped['pose']
            json_header = json_posestamped['header']
            json_pos = json_pose['position']
            json_ori = json_pose['orientation']
            json_stamp = json_header['stamp']

            if adversary in self.seq_adversaries and self.seq_adversaries[adversary] == json_header['seq']:
                continue
            self.seq_adversaries[adversary] = json_header['seq']

            pos = [json_pos['x'], json_pos['y'], json_pos['z']]
            quat = [json_ori['x'], json_ori['y'], json_ori['z'], json_ori['w']]
            # stamp = rospy.Time(json_stamp['secs'], json_stamp['nsecs'])
            stamp = rospy.get_rostime()

            print("Publish {}({}.{}): {}, {}".format(adversary, stamp.secs, stamp.nsecs, pos, quat))
            self.tf_pub.sendTransform(pos, quat, stamp, adversary, 'map')

    def run(self):
        """
        Relay poses to and from Redis.
        """
        print("Running")
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.publish_pose_to_redis()
            self.get_poses_from_redis()
            rate.sleep()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('redis', help="Hostname of Redis master")
    parser.add_argument('--hostname', default=socket.gethostname(), help="Hostname of this turtlebot")
    args = parser.parse_args(rospy.myargv()[1:])

    r = RedisRelay(args.redis, args.hostname)
    r.run()
