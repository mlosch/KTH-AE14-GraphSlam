#!/usr/bin/python2
import sys
import tf
import rosbag
import math

def anglediff(a,b):
	return (b-a+math.pi) % (2*math.pi) - math.pi

def posediff(a,b):
	return (b[0]-a[0], b[1]-a[1], anglediff(a[2],b[2]))

def msg2pose(msg):
	pose = msg.pose.pose
	x = pose.position.x
	y = pose.position.y
	o = pose.orientation
	theta = tf.transformations.euler_from_quaternion((o.x,o.y,o.z,o.w))[2]
	return (x,y,theta)

def msg2dist(msg):
	return (msg.front, msg.rear, msg.left_front, msg.left_rear, msg.right_front, msg.right_rear)


if len(sys.argv) < 2:
	print 'usage:', sys.argv[0], 'bag'
	exit()



print 'delta_pose,ir'

pose = (0,0,0)
prev_pose = False

bag = rosbag.Bag(sys.argv[1])
for topic, msg, t in bag.read_messages(topics=['/sensors/pose', '/sensors/ir/distances']):
	if topic == '/sensors/pose':
		pose = msg2pose(msg)
		if prev_pose == False:
			prev_pose = pose
	elif topic == '/sensors/ir/distances':
		pose_delta = posediff(pose,prev_pose);
		dist = msg2dist(msg)
		print '[%9.6f %9.6f %9.6f],\t[%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f]' % (pose_delta + dist)
		prev_pose = pose
bag.close()
