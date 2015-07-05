#!/usr/bin/python

import rospy
import time

from kbhit import getch, kbhit, kbhit_init
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState


#constants
NODE_REFRESH_FREQ = 20.0	# Hz
NODE_NAME = 'reactor_teleop'

joint_list = (
	'shoulder_yaw',
	'shoulder_pitch',
	'elbow_pitch',
	'wrist_pitch',
	'wrist_roll',
)

LEGEND = 'SHOULDER Y: 1,2;  SHOULDER P: 3,4;  ELBOW P: 5,6;  WRIST P: 7,8;  WRIST R: 9,0'

key_map = { '1': ( 0, False ),
			'2': ( 0, True ),
			'3': ( 1, False ),
			'4': ( 1, True ),
			'5': ( 2, False ),
			'6': ( 2, True ),
			'7': ( 3, False ),
			'8': ( 3, True ),
			'9': ( 4, False ),
			'0': ( 4, True ),
		  }


#global variables
joint_states_dict = dict()
listeners_dict = dict()
publishers_dict = dict()
publishers_value_dict = dict()

'''
header: 
  seq: 27
  stamp: 
    secs: 1436102611
    nsecs: 218693971
  frame_id: ''
name: wrist_pitch
motor_ids: [6]
motor_temps: [57]
goal_pos: 0.598252507275
current_pos: 0.608479045861
error: 0.0102265385859
velocity: 0.0
load: 0.0625
is_moving: False
'''

def joint_state_callback( msg ):
	'''
	callback to capture ros publication
	'''

	joint_states_dict[ msg.name ] = msg.current_pos

def main():
	kbhit_init()

#	import pdb
#	pdb.set_trace()

	rospy.init_node( NODE_NAME )
	rate = rospy.Rate( NODE_REFRESH_FREQ ) # max frequency of sending target joint positions

	# set subscribers
	for j in joint_list:
		listeners_dict[ j ] = rospy.Subscriber( '/' + j + '_controller/state', JointState, joint_state_callback )

	# set publishers
	for j in joint_list:
		publishers_dict[ j ] = rospy.Publisher( '/' + j + '_controller/command', Float64, queue_size=10 )

	# get initial joint states
	for j in joint_list:
		while j not in joint_states_dict:
			print( 'waiting for: ' + j )
			time.sleep( 0.5 )
		publishers_value_dict[ j ] = joint_states_dict[ j ]


	print( LEGEND )
	print( joint_states_dict )

	while not rospy.is_shutdown():

		# broadcast value
		for t in publishers_dict:
			publishers_dict[ t ].publish( Float64( publishers_value_dict[ t ] ) )

		rate.sleep()

		# check for key
		if not kbhit():
			continue
		
		# got key
		k = getch()
			
		# check if key in dictionary
		if k in key_map:
			topic_num, up_down = key_map[ k ]
			topic = joint_list[ topic_num ]
			
			p = publishers_value_dict[ topic ]
			if up_down:
				publishers_value_dict[ topic ] = p + 0.01
			else:
				publishers_value_dict[ topic ] = p - 0.01

			print( topic, publishers_value_dict[ topic ] )

	
	print( 'shutting down teleop' )


if __name__ == '__main__':
	main()
