import mc_control
import mc_rbdyn
import mc_rtc
import ast

import rospy
from check_selfcollision.srv import *

import numpy as np

class PythonExampleController(mc_control.MCPythonController):
	def __init__(self, rm, dt):
		self.dt = dt
		self.config().load('/home/laptop/mc_playground/controllers/python_example_controller/etc/PythonExampleController.in.yaml')
		config_ = self.config()
		self.jointNames =  ast.literal_eval(config_("ControllerParams")("JointNames").dump(False).decode())
		self.upper = ast.literal_eval(config_("ControllerParams")("UpperLimits").dump(False).decode())
		self.lower = ast.literal_eval(config_("ControllerParams")("LowerLimits").dump(False).decode())
		self.errorTolerance = ast.literal_eval(config_("ControllerParams")("AngleErrorTolerance").dump(False).decode())
		self.mbcJointIdx = {}
		refJointIdx = {}
		self.rand_multiplier = []
		for i in range(len(self.jointNames)):
			name = self.jointNames[i]
			print(name, self.lower[i], self.upper[i])
			self.mbcJointIdx[name] = self.robot().jointIndexByName(name)
			print("PythonExampleController Joint["+str(i)+"] Name: "+name+" mbc index:" + str(self.mbcJointIdx[name]))
			self.rand_multiplier.append((self.upper[i] - self.lower[i]))

		rospy.init_node('PythonExampleController', anonymous=True)
		print("PythonExampleController: ROS init")

		self.switch_target()
		self.qpsolver.addConstraintSet(self.selfCollisionConstraint)
		self.qpsolver.addTask(self.postureTask)

		print("PythonExampleController: init done")
	
	def run_callback(self):
		if rospy.is_shutdown():
			return False
		# target = self.postureTask.posture()
		try:
			mc_rbdyn
			# self.robot().eulerIntegration(self.robot().mbc(), self.dt)
			q = self.robot().q
			# print("PythonExampleController", [i[0] for i in q[self.mbcJointIdx[self.jointNames[0]]:self.mbcJointIdx[self.jointNames[-1]]+1]])
			# alpha = self.robot().alpha()
			# alphaD = self.robot().alphaD()
			# jointTorque = self.robot().jointTorque()
			# encoderValues = self.robot().encoderValues()
			return self.qpsolver.run()
			# return True
		except Exception as e:
			print("PythonExampleController: Encountered Exception in run", str(e))
		return False
	
	def reset_callback(self, data):
		pass
	
	@staticmethod
	def create(robot, dt):
		env = mc_rbdyn.get_robot_module("env", mc_rtc.MC_ENV_DESCRIPTION_PATH, "ground")
		return PythonExampleController([robot,env], dt)
	
	def switch_target(self):
		is_colliding = True
		service = rospy.ServiceProxy('check_selfcollision_service', SelfCollosion)
		request = SelfCollosionRequest()
		targets = {}
		while(is_colliding):
			request.robot_state.joint_state.position.clear()
			request.robot_state.joint_state.name.clear()
			for i in range(len(self.jointNames)):
				value = [self.lower[i] + self.rand_multiplier[i]*np.random.random()]
				targets[str.encode(self.jointNames[i])] = value
				request.robot_state.joint_state.name.append(self.jointNames[i])
				request.robot_state.joint_state.position.append(value[0])
			try:
				response = service(request)
				is_colliding = response.is_colliding
				print("PythonExampleController: Collision Check Result:", is_colliding, targets[str.encode(self.jointNames[0])][0], targets[str.encode(self.jointNames[1])][0], targets[str.encode(self.jointNames[2])][0], targets[str.encode(self.jointNames[3])][0])
			except rospy.ServiceException as e: 
				print("PythonExampleController: Collision Check Failed")
				raise rospy.ServiceException()
			
		self.postureTask.target(targets)
		print("PythonExampleController: New Posture Target:", targets[str.encode(self.jointNames[0])][0], targets[str.encode(self.jointNames[1])][0], targets[str.encode(self.jointNames[2])][0], targets[str.encode(self.jointNames[3])][0])
		print("PythonExampleController: Switch Target")
