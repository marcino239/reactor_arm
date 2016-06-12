import gflags
import dynamixel
import time
import sys
import pygame

SERVO_STEP = 10


FLAGS = gflags.FLAGS
gflags.DEFINE_string( 'port', '/dev/ttyUSB0', 'dynamixel port' )
gflags.DEFINE_integer( 'baud', 1000000, 'baud rate' )
gflags.DEFINE_integer( 'min_id', 1, 'lowest dynamixel ID' )
gflags.DEFINE_integer( 'max_id', 8, 'highest dynamixel ID' )
gflags.DEFINE_enum( 'command', '', [ '', 'position', 'torque_on', 'torque_off', 'control' ], 'command to execute' )


if __name__ == '__main__':
	
	flags = FLAGS( sys.argv )
	
	serial = dynamixel.SerialStream( port=FLAGS.port,
										baudrate=FLAGS.baud,
										timeout=1 )
	# Instantiate network object
	net = dynamixel.DynamixelNetwork( serial )
	
	# Populate network with dynamixel objects
	for servoId in range( FLAGS.min_id, FLAGS.max_id + 1 ):
		newDynamixel = dynamixel.Dynamixel( servoId, net )
		net._dynamixel_map[ servoId ] = newDynamixel
		
	servos = net.get_dynamixels()
	print( 'network initialised' )

	if FLAGS.command == '' or FLAGS.command == 'torque_off':
		for d in servos:
			d.torque_enable = False

	elif FLAGS.command == 'position':
		while True:
			pos = [ d.current_position for d in servos ]
			pos_txt = [ '{:4d}'.format( p ) for p in pos ]
			print( ''.join( pos_txt ) )
			time.sleep( 0.25 )

	elif FLAGS.command == 'torque_off':
		for d in servos:
			d.torque_enable = True

	elif FLAGS.command == 'control':
		
		pygame.init()
		pygame.joystick.init()
		js = pygame.joystick.Joystick( 0 )
		js.init()

		# torque on
		for d in servos:
			d.moving_speed = 50
			d.torque_enable = True
			d.torque_limit = 800 
			d.max_torque = 800
			d.goal_position = 512

		# Send all the commands to the servos.
		net.synchronize()

		print( 'moving to default position' )
		time.sleep( 5 )
		print( 'done' )

		# get initial positions
		servo_pos = [ d.current_position for d in servos ]

		clip = lambda x: int( min( 1023.0, max( 0.0, x ) ) )

		while True:
			pygame.event.pump()
			axis = [ js.get_axis( a ) for a in range( 27 ) ]
			
			servo_pos[0] = clip( servo_pos[0] - axis[0] * SERVO_STEP )
			servo_pos[1] = clip( servo_pos[1] + axis[1] * SERVO_STEP )
			servo_pos[2] = clip( servo_pos[2] + axis[3] * SERVO_STEP )
			servo_pos[3] = clip( servo_pos[3] + axis[2] * SERVO_STEP )
			servo_pos[4] = clip( servo_pos[4] - (axis[12] + 1.0) * SERVO_STEP / 2 + (axis[13] + 1.0) * SERVO_STEP / 2 )
			servo_pos[5] = clip( servo_pos[5] - (axis[14] + 1.0) * SERVO_STEP / 2 + (axis[15] + 1.0) * SERVO_STEP / 2 )
			
			if axis[0] != 0.0:
				# shoulder yaw
				servos[0].goal_position = servo_pos[0]
			else:
				if abs( servos[0].current_position - servo_pos[0] ) > SERVO_STEP:
					servo_pos[0] = servos[0].current_position
			
			if axis[1] != 0.0:
				# shoulder piych - coupling
				servos[1].goal_position = servo_pos[1]
				servos[2].goal_position = 1024 - servo_pos[1]
			else:
				if abs( servos[1].current_position - servo_pos[1] ) > SERVO_STEP:
					servo_pos[1] = servos[1].current_position 
			
			if axis[3] != 0.0:
				# elbow pitch - coupling
				servos[3].goal_position = servo_pos[2]
				servos[4].goal_position = 1024 - servo_pos[2]
			else:
				if abs( servos[3].current_position - servo_pos[2] ) > SERVO_STEP:
					servo_pos[2] = servos[3].current_position 
			
			if axis[2] != 0.0:
				# wrist pitch
				servos[5].goal_position = servo_pos[3]
			else:
				if abs( servos[5].current_position - servo_pos[3] ) > SERVO_STEP:
					servo_pos[3] = servos[5].current_position 

			# wrist roll
			servos[6].goal_position = servo_pos[4]

			# gripper
			servos[7].goal_position = servo_pos[5]

			# show desired position
#			print( ''.join( [ '{:4d}'.format( p ) for p in servo_pos ] ) )

			# current position
#			print( ''.join( [ '{:5d}'.format( d.current_position ) for d in servos ] ) )

			# goal position
#			print( ''.join( [ '{:5d}'.format( d.goal_position ) for d in servos ] ) )

			# diff position
#			print( 'diff: ' + ''.join( [ '{:5d}'.format( d.current_position - d.goal_position ) for d in servos ] ) )

			# current temperature
#			print( ''.join( [ '{:3d}'.format( d.current_temperature ) for d in servos ] ) )

			# current load
#			print( ''.join( [ '{:5d}'.format( d.current_load ) for d in servos ] ) )

			# current load and temperature
			print( ''.join( [ '{:5d},{:3d} '.format( d.current_load, d.current_temperature ) for d in servos ] ) )

			# Send all the commands to the servos.
			net.synchronize()

			time.sleep( 0.05 )
