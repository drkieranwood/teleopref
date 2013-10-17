#!/usr/bin/env python
import roslib; roslib.load_manifest('teleopref')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to topic ref_pose
---------------------------
up/down:       move forward/backward
left/right:    move left/right
w/s:           increase/decrease altitude
a/d:           turn left/right
t/l:           takeoff/land
r:             reset (toggle emergency state)
anything else: n/a

please don't have caps lock on.
CTRL+c to quit
"""



#The key bindings. The magnitude of the change can be set here
move_bindings = {
		68:('linear', 'y', 0.06), #left
		67:('linear', 'y', -0.06), #right
		65:('linear', 'x', 0.06), #forward
		66:('linear', 'x', -0.06), #back
		'w':('linear', 'z', 0.01),
		's':('linear', 'z', -0.01),
		'a':('angular', 'z', 1),
		'd':('angular', 'z', -1),
	       }

#Function to get a key press from the system
def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	print msg

	#Storage for the current reference position
	XX=0.0
	YY=0.0
	ZZ=0.0
	WW=0.0

	pub = rospy.Publisher('ref_pose', Twist)
	land_pub = rospy.Publisher('/ardrone/land', Empty)
	reset_pub = rospy.Publisher('/ardrone/reset', Empty)
	takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty)

	rospy.init_node('teleopref')

	twistinit = Twist()
	twistinit.linear.x = XX; twistinit.linear.y = YY; twistinit.linear.z = ZZ
	twistinit.angular.x = 0; twistinit.angular.y = 0; twistinit.angular.z = WW
	print twistinit
	pub.publish(twistinit)

	try:
		while(True):
			# Retrieve the latest key press from the system. 
			# This will pause the node until a key is pressed.
			key = getKey()

			# If it is a (l, r, or t) takeoff and landing or emergency
			if key == 'l':
				land_pub.publish(Empty())
			if key == 'r':
				reset_pub.publish(Empty())
			if key == 't':
				takeoff_pub.publish(Empty())
			# If it is the escape key then get another two key presses
			# Dunno why this is in here?
                        if ord(key) == 27:
                            key = getKey()
                            key = getKey()

			# Create output message variable
			twist = Twist()

			# If the unicode of the pressed key appears in the list, 
			# then convert it to the unicode value permanently.
			if ord(key) in move_bindings.keys():
                                key = ord(key)
			
			
			# If the literal/unicode appear in the list above then
			# put the value into the twist message.
			if key in move_bindings.keys():
				(lin_ang, xyz, speed) = move_bindings[key]
				#setattr(getattr(twist, lin_ang), xyz, speed)
				#print key
				if key in (68,67):
					YY=YY-speed
				elif key in (65,66):
					XX=XX+speed
				elif key in ('w','s'):
					ZZ=ZZ-speed
				elif key in ('a','d'):
					WW=WW-speed

				twist.linear.x = XX; twist.linear.y = YY; twist.linear.z = ZZ
				twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = ((WW/(180.0))*(3.14159))
			
			# If the ctrl+c has been called then break the loop.
			else:
				if (key == '\x03'):
					break
			
			# Publish the message 
			pub.publish(twist)
			twist.angular.z = WW
			print twist

	except Exception as e:
		print e
		print repr(e)

	finally:
		#Once the above loop has been broken, kill the node with 
		#a zero command
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


