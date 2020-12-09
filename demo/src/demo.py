#!/usr/bin/env python

import rospy,actionlib
from iiwa_msgs import msg, srv
import actionlib_msgs.msg
import demo_msgs.srv


def create_movement(link,position_x,position_y,position_z,orientation_x,orientation_y,orientation_z,orientation_w):

	movement=msg.CartesianPose()
	
	movement.poseStamped.header.seq = 1 
	movement.poseStamped.header.stamp = rospy.Time.now()
	movement.poseStamped.header.frame_id = link
	
	movement.poseStamped.pose.position.x = position_x
	movement.poseStamped.pose.position.y = position_y
	movement.poseStamped.pose.position.z = position_z

	movement.poseStamped.pose.orientation.x = orientation_x
	movement.poseStamped.pose.orientation.y = orientation_y
	movement.poseStamped.pose.orientation.z = orientation_z
	movement.poseStamped.pose.orientation.w = orientation_w

	movement.redundancy.status = -1
	movement.redundancy.e1 = -1 
	
	return movement

"""
Configure gripper and configure led services values
"""
OPEN,CLOSE=True,False 

ON,OFF=True,False
RED,GREEN,BLUE=1,2,3
BLINK_ON,BLINK_OFF= True,False

def configure_gripper(action):
	gripper = rospy.ServiceProxy('/iiwa/configuration/openGripper',srv.OpenGripper)
	gripper_request = srv.OpenGripperRequest(action)		
	gripper_response = gripper(gripper_request) 
	rospy.loginfo(gripper_response)

def configure_led(on, color, blinking):
	configure_led = rospy.ServiceProxy('/iiwa/configuration/configureLed',srv.ConfigureLed)	
	configure_led_request = srv.ConfigureLedRequest(on,color,blinking)
	configure_led_response = configure_led(configure_led_request)
	rospy.loginfo(configure_led_response)

def get_pallet_movement(areaX):
	if(areaX==1.0):
		movement=create_movement('iiwa_link_0',0.67899,-0.181,0.2,0,1,0,0)
		return movement
	elif(areaX==2.0):
		movement=create_movement('iiwa_link_0',0.67899,-0.244,0.2,0,1,0,0)
		return movement
	elif(areaX==3.0):
		movement=create_movement('iiwa_link_0',0.67899,-0.305,0.2,0,1,0,0)
		return movement
	else:
		rospy.logerr("[pallet]selected area does not exits")

def get_buffer_movement(areaX):
	if(areaX==1.0):
		movement=create_movement('iiwa_link_0',0.41899,0.07,0.133,0,1,0,0)
		return movement
	elif(areaX==2.0):
		movement=create_movement('iiwa_link_0',0.41899,-0.02,0.133,0,1,0,0)
		return movement
	elif(areaX==3.0):
		movement=create_movement('iiwa_link_0',0.41899,-0.11,0.133,0,1,0,0)
		return movement
	elif(areaX==4.0):
		movement=create_movement('iiwa_link_0',0.41899,-0.20,0.133,0,1,0,0)
		return movement
	elif(areaX==5.0):
		movement=create_movement('iiwa_link_0',0.41899,-0.29,0.133,0,1,0,0)
		return movement
	elif(areaX==6.0):
		movement=create_movement('iiwa_link_0',0.41899,-0.380,0.133,0,1,0,0)
		return movement
	else:
		rospy.logerr("[buffer]selected area does not exits")

def execute_pick(client,pick_areaID,pick_areaX,pick_areaY):
	
	movement = None
	
	if (pick_areaID == 1.0):
		movement = get_pallet_movement(pick_areaX)
	elif(pick_areaID == 2.0):
		movement = get_buffer_movement(pick_areaX)
	else:
		rospy.logerr("[pick]selected area does not exist")

	"""
	HIGH POINT
	"""
	movement.poseStamped.pose.position.z +=0.2
	action_goal = msg.MoveToCartesianPoseGoal(movement)
	client.send_goal_and_wait(action_goal)					
	client.wait_for_result()


	"""
	LOW POINT
	"""
	movement.poseStamped.pose.position.z -=0.2
	action_goal = msg.MoveToCartesianPoseGoal(movement)
	client.send_goal_and_wait(action_goal)					
	client.wait_for_result()

	"""
	CLOSE GRIPPER
	"""
	configure_led(ON,BLUE,BLINK_ON)
	configure_gripper(CLOSE)
	configure_led(ON,GREEN,BLINK_OFF)

	"""
	RET POINT
	"""
	movement.poseStamped.pose.position.z +=0.2
	action_goal = msg.MoveToCartesianPoseGoal(movement)
	client.send_goal_and_wait(action_goal)					
	client.wait_for_result()


def execute_place(client,place_areaID,place_areaX,place_areaY):
	
	movement = None

	if (place_areaID == 1.0):
		movement = get_pallet_movement(place_areaX)
	elif(place_areaID == 2.0):
		movement = get_buffer_movement(place_areaX)
	else:
		rospy.logerr("[place]selected area does not exist")

	"""
	HIGH POINT
	"""
	movement.poseStamped.pose.position.z+=0.2
	action_goal = msg.MoveToCartesianPoseGoal(movement)
	client.send_goal_and_wait(action_goal)					
	client.wait_for_result()

	"""
	LOW POINT
	"""
	movement.poseStamped.pose.position.z-=0.2
	action_goal = msg.MoveToCartesianPoseGoal(movement)
	client.send_goal_and_wait(action_goal)					
	client.wait_for_result()
	"""
	OPEN GRIPPER
	"""
	configure_led(ON,BLUE,BLINK_ON)
	configure_gripper(OPEN)
	configure_led(ON,GREEN,BLINK_OFF)

	"""
	RET POINT
	"""
	movement.poseStamped.pose.position.z+=0.2
	action_goal = msg.MoveToCartesianPoseGoal(movement)
	client.send_goal_and_wait(action_goal)					
	client.wait_for_result()

def go_home(client):

	movement = create_movement('iiwa_link_0',0.31899,-0.39,0.5,0,1,0,0)		
	action_goal = msg.MoveToCartesianPoseGoal(movement)
	client.send_goal_and_wait(action_goal)					
	client.wait_for_result()	


def demo(pick_areaID,pick_areaX,pick_areaY,place_areaID,place_areaX,place_areaY):

	client = actionlib.SimpleActionClient('/iiwa/action/move_to_cartesian_pose', msg.MoveToCartesianPoseAction)
	client.wait_for_server()	
	client.cancel_all_goals()		

	"""INIT CONF"""
	configure_led(OFF,GREEN,BLINK_OFF)
	configure_gripper(OPEN)
	configure_led(ON,GREEN,BLINK_OFF)

	"""
	HOME POSITION
	"""
	go_home(client)
	"""
	PICK
	"""
	execute_pick(client,pick_areaID,pick_areaX,pick_areaY)

	"""
	PLACE
	"""
	execute_place(client,place_areaID,place_areaX,place_areaY)

	"""
	HOME POSITION
	"""
	go_home(client)

	"""FINAL BLINKING"""
	configure_led(ON,RED,BLINK_ON)
	rospy.sleep(2)
	configure_led(OFF,RED,BLINK_ON)
	rospy.loginfo('task completed')


def execute_demo(request):
	demo(request.pick_areaID,request.pick_areaX,request.pick_areaY,request.place_areaID,request.place_areaX,request.place_areaY)
	response = demo_msgs.srv.OpcuaServiceResponse(True)
	return response

if __name__ == '__main__':
	rospy.init_node('demo_using_actions')
	
	"""
	Waits for led and gripper services to be available
	"""
	rospy.wait_for_service('/iiwa/configuration/configureLed')
	rospy.wait_for_service('/iiwa/configuration/openGripper')

	"""
	Declares service
	"""
	rospy.Service("/iiwa/demo",demo_msgs.srv.OpcuaService,execute_demo)
	
	rospy.spin()