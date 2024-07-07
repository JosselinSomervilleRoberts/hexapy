#! /usr/bin/env python
#| This file is a part of the pyite framework.
#| Copyright 2019, INRIA
#| Main contributor(s):
#| Jean-Baptiste Mouret, jean-baptiste.mouret@inria.fr
#| Eloise Dalin , eloise.dalin@inria.fr
#| Pierre Desreumaux , pierre.desreumaux@inria.fr
#|
#| Antoine Cully, Jeff Clune, Danesh Tarapore, and Jean-Baptiste Mouret.
#|"Robots that can adapt like animals." Nature 521, no. 7553 (2015): 503-507.
#|
#| This software is governed by the CeCILL license under French law
#| and abiding by the rules of distribution of free software.  You
#| can use, modify and/ or redistribute the software under the terms
#| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
#| following URL "http://www.cecill.info".
#|
#| As a counterpart to the access to the source code and rights to
#| copy, modify and redistribute granted by the license, users are
#| provided only with a limited warranty and the software's author,
#| the holder of the economic rights, and the successive licensors
#| have only limited liability.
#|
#| In this respect, the user's attention is drawn to the risks
#| associated with loading, using, modifying and/or developing or
#| reproducing the software by the user in light of its specific
#| status of free software, that may mean that it is complicated to
#| manipulate, and that also therefore means that it is reserved for
#| developers and experienced professionals having in-depth computer
#| knowledge. Users are therefore encouraged to load and test the
#| software's suitability as regards their requirements in conditions
#| enabling the security of their systems and/or data to be ensured
#| and, more generally, to use and operate it in the same conditions
#| as regards security.
#|
#| The fact that you are presently reading this means that you have
#| had knowledge of the CeCILL license and that you accept its terms.
import os
import time
import math
from timeit import default_timer as timer
import subprocess 
import time
import numpy as np
import pybullet as p
import pybullet_utils.bullet_client as bc
import pybullet_data
from urdf_parser import parse_urdf_file

from pycontrollers.hexapod_controller import HexapodController
from hexapod import Hexapod

class HexapodSimulator:
	def __init__(self, hexapod: Hexapod,
			  	gui: bool, 
				urdf: str, 
				dt = 1./240.,  # the default for pybullet (see doc)
				control_dt=0.05,
				video=''):
		self.hexapod = hexapod
		self.GRAVITY = -9.81
		self.dt = dt
		self.control_dt = control_dt
		# we call the controller every control_period steps
		self.control_period = int(control_dt / dt)
		self.t = 0
		self.i = 0
		self.safety_turnover = True
		self.video_output_file = video

		# the final target velocity is computed using:
		# kp*(erp*(desiredPosition-currentPosition)/dt)+currentVelocity+kd*(m_desiredVelocity - currentVelocity).
		# here we set kp to be likely to reach the target position
		# in the time between two calls of the controller
		self.kp = 2./12.# * self.control_period
		self.kd = 0.4
		# the desired position for the joints
		self.angles = np.array(self.hexapod.get_servo_angles())
		# setup the GUI (disable the useless windows)
		if gui:
			self.physics = bc.BulletClient(connection_mode=p.GUI)
			self.physics.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
			self.physics.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
			self.physics.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
			self.physics.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
			self.physics.resetDebugVisualizerCamera(cameraDistance=1,
													cameraYaw=20,
										 			cameraPitch=-20,
													cameraTargetPosition=[1, -0.5, 0.8])
			self.keys_pressed = set()
		else:
			self.physics = bc.BulletClient(connection_mode=p.DIRECT)

		self.physics.setAdditionalSearchPath(pybullet_data.getDataPath())
		self.physics.resetSimulation()
		self.physics.setGravity(0,0,self.GRAVITY)
		self.physics.setTimeStep(self.dt)
		self.physics.setPhysicsEngineParameter(fixedTimeStep=self.dt)
		self.planeId = self.physics.loadURDF("plane.urdf")
		self.set_friction(self.planeId, lateral_friction=1.0)  # Set the desired friction coefficient

		start_pos = self.hexapod.get_pos() + np.array([0, 0, 1.])
		start_orientation = self.physics.getQuaternionFromEuler([0.,0,0])
		self.botId = self.physics.loadURDF(urdf, basePosition=start_pos, baseOrientation=start_orientation, flags=(p.URDF_USE_SELF_COLLISION))
		self.joint_list = self._make_joint_list(self.botId)

		# bullet links number corresponding to the legs
		self.leg_link_ids = [17, 14, 2, 5, 8, 11]
		self.set_robot_feet_friction(self.botId, lateral_friction=1.0)  # Set the desired friction coefficient
		self.descriptor = {17 : [], 14 : [], 2 : [], 5 : [], 8 : [], 11 : []}

		# video makes things much slower
		if (video != ''):
			self._stream_to_ffmpeg(self.video_output_file)

		# put the hexapod on the ground (gently)
		self.physics.setRealTimeSimulation(0)
		jointFrictionForce=1

		self.angles = self.hexapod.get_servo_angles()
		for joint in range (self.physics.getNumJoints(self.botId)):
			self.physics.setJointMotorControl2(self.botId, joint,
				p.POSITION_CONTROL,
				force=jointFrictionForce)
		for t in range(0, 100):
			self.physics.stepSimulation()
			# move the joints
			missing_joint_count = 0
			j = 0
			for joint in self.joint_list:
				if(joint==1000):
					missing_joint_count += 1
				else:
					info = self.physics.getJointInfo(self.botId, joint)
					lower_limit = info[8]
					upper_limit = info[9]
					max_force = info[10]
					max_velocity = info[11]
					pos = min(max(lower_limit, self.angles[j]), upper_limit)
					self.physics.setJointMotorControl2(self.botId, joint,
						p.POSITION_CONTROL,
						positionGain=self.kp,
						velocityGain=self.kd,
						targetPosition=pos,
						force=max_force,
						maxVelocity=max_velocity)
				j += 1
			self.physics.setGravity(0,0, self.GRAVITY)

	def set_friction(self, body_id, lateral_friction):
		self.physics.changeDynamics(body_id, -1, lateralFriction=lateral_friction)

	def set_robot_feet_friction(self, robot_id, lateral_friction):
		for leg_link_id in self.leg_link_ids:
			self.physics.changeDynamics(robot_id, leg_link_id, lateralFriction=lateral_friction)


	def get_state(self):
		# Get joint infos
		self.joint_torques = np.zeros(len(self.joint_list))
		self.joint_angles = np.zeros(len(self.joint_list))
		self.joint_velocities = np.zeros(len(self.joint_list))
		for i in range(len(self.joint_list)):
			joint = self.joint_list[i]
			pos, vel, _, torque = self.physics.getJointState(self.botId, joint)
			self.joint_angles[i] = pos
			self.joint_velocities[i] = vel
			self.joint_torques[i] = torque

		return self.get_pos(), self.joint_angles, self.joint_velocities, self.joint_torques



	def destroy(self):
		try:
			self.physics.disconnect()
			if self.video_output_file != '':
				self.ffmpeg_pipe.stdin.close()
				self.ffmpeg_pipe.stderr.close()
				self.ffmpeg_pipe.wait()
		except p.error as e:
			print("Warning (destructor of simulator):", e)


	def reset(self):
		assert(0), "not working for now"
		self.t = 0
		self.physics.resetSimulation()
#		self.physics.restoreState(self._init_state)
		

	def get_pos(self):
		'''
		Returns the position list of 3 floats and orientation as list of 4 floats in [x,y,z,w] order.
		Use p.getEulerFromQuaternion to convert the quaternion to Euler if needed.
		'''
		return self.physics.getBasePositionAndOrientation(self.botId)

	def step(self, controller):
		if self.i % self.control_period == 0:
			self.angles = controller.step(self)
		self.i += 1
		
		# 24 FPS dt =1/240 : every 10 frames
		if self.video_output_file != '' and self.i % (int(1. / (self.dt * 24))) == 0: 
			camera = self.physics.getDebugVisualizerCamera()
			img = p.getCameraImage(camera[0], camera[1], renderer=p.ER_BULLET_HARDWARE_OPENGL)
			self.ffmpeg_pipe.stdin.write(img[2].tobytes())

		#Check if roll pitch are not too high
		error = False
		self.euler = self.physics.getEulerFromQuaternion(self.get_pos()[1])
		if(self.safety_turnover):
			if((abs(self.euler[1]) >= math.pi/2) or (abs(self.euler[0]) >= math.pi/2)):
				error = True

		# move the joints
		missing_joint_count = 0
		j = 0
		for joint in self.joint_list:
			if(joint==1000):
				missing_joint_count += 1
			else:
				info = self.physics.getJointInfo(self.botId, joint)
				lower_limit = info[8]
				upper_limit = info[9]
				max_force = info[10]
				max_velocity = info[11]
				pos = min(max(lower_limit, self.angles[j]), upper_limit)
				self.physics.setJointMotorControl2(self.botId, joint,
					p.POSITION_CONTROL,
					positionGain=self.kp,
					velocityGain=self.kd,
					targetPosition=pos,
					force=max_force,
					maxVelocity=max_velocity)
			j += 1

		#Get contact points between robot and world plane
		contact_points = self.physics.getContactPoints(self.botId,self.planeId)
		link_ids = [] #list of links in contact with the ground plane
		if(len(contact_points) > 0):
			for cn in contact_points:
				linkid= cn[3] #robot link id in contact with world plane
				if linkid not in link_ids:
					link_ids.append(linkid)
		for l in self.leg_link_ids:
			cns = self.descriptor[l]
			if l in link_ids:
				cns.append(1)
			else:
				cns.append(0)
			self.descriptor[l] = cns
		# print(f"Contact links: {link_ids}")

		# don't forget to add the gravity force!
		self.physics.setGravity(0, 0, self.GRAVITY)

		# finally, step the simulation
		self.physics.stepSimulation()
		self.t += self.dt
		return error

	def get_joints_positions(self):
		''' return the actual position in the physics engine'''
		p = np.zeros(len(self.joint_list))
		i = 0
		# be careful that the joint_list is not necessarily in the same order as 
		# in bullet (see make_joint_list)
		for joint in self.joint_list:
			p[i] = self.physics.getJointState(self.botId, joint)[0]
			i += 1
		return p


	def _stream_to_ffmpeg(self, fname):
		camera = self.physics.getDebugVisualizerCamera()
		command = ['ffmpeg',
				'-y',
				'-f', 'rawvideo',
				'-vcodec','rawvideo',
				'-s',  '{}x{}'.format(camera[0], camera[1]),
				'-pix_fmt', 'rgba',
				'-r', str(24),
				'-i', '-',
				'-an',
				'-vcodec', 'mpeg4',
				'-vb', '20M',
				fname]
		print(command)
		self.ffmpeg_pipe = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.PIPE)

	def _make_joint_list(self, botId):
		joint_names = [b'body_leg_0', b'leg_0_1_2', b'leg_0_2_3',
		b'body_leg_1', b'leg_1_1_2', b'leg_1_2_3',
		b'body_leg_2', b'leg_2_1_2', b'leg_2_2_3',
		b'body_leg_3', b'leg_3_1_2', b'leg_3_2_3',
		b'body_leg_4', b'leg_4_1_2', b'leg_4_2_3',
		b'body_leg_5', b'leg_5_1_2', b'leg_5_2_3',
		]
		joint_list = []
		for n in joint_names:
			joint_found = False
			for joint in range (self.physics.getNumJoints(botId)):
				name = self.physics.getJointInfo(botId, joint)[1]
				if name == n:
					joint_list += [joint]
					joint_found = True
			if(joint_found==False):
				joint_list += [1000] #if the joint is not here (aka broken leg case) put 1000
		return joint_list
	
	def handle_key_events(self, controller):
		keys = self.physics.getKeyboardEvents()
		for k, v in keys.items():
			if v & p.KEY_IS_DOWN:
				self.keys_pressed.add(chr(k))
			elif v & p.KEY_WAS_RELEASED:
				self.keys_pressed.remove(chr(k))

		direction = 0
		speed = 0
		phi_speed = 0
		PHI_SPEED = 1.0
		SPEED = 0.3
		if "w" in self.keys_pressed:
			direction = np.pi/2
			speed = SPEED
		elif "s" in self.keys_pressed:
			direction = -np.pi/2
			speed = SPEED
		if "a" in self.keys_pressed:
			direction = np.pi
			speed = SPEED
		elif "d" in self.keys_pressed:
			direction = 0
			speed = SPEED
		if "j" in self.keys_pressed:
			phi_speed = PHI_SPEED
		elif "k" in self.keys_pressed:
			phi_speed = -PHI_SPEED
		controller.direction = direction
		controller.speed = speed
		controller.phi_speed = phi_speed

	def move_robot(self, key):
		force = 10
		if key == ord('w'):
			self.physics.applyExternalForce(self.botId, -1, [force, 0, 0], [0, 0, 0], p.WORLD_FRAME)
		elif key == ord('s'):
			self.physics.applyExternalForce(self.botId, -1, [-force, 0, 0], [0, 0, 0], p.WORLD_FRAME)
		elif key == ord('a'):
			self.physics.applyExternalTorque(self.botId, -1, [0, 0, force], p.WORLD_FRAME)
		elif key == ord('d'):
			self.physics.applyExternalTorque(self.botId, -1, [0, 0, -force], p.WORLD_FRAME)

	def control_camera(self, key):
		current_cam_pos, current_cam_target, current_cam_up, current_cam_yaw, current_cam_pitch, current_cam_roll, current_cam_dist = self.physics.getDebugVisualizerCamera()
		if key == ord('i'):
			current_cam_pos[2] += 0.1
		elif key == ord('k'):
			current_cam_pos[2] -= 0.1
		elif key == ord('j'):
			current_cam_pos[0] -= 0.1
		elif key == ord('l'):
			current_cam_pos[0] += 0.1
		self.physics.resetDebugVisualizerCamera(current_cam_dist, current_cam_yaw, current_cam_pitch, current_cam_pos)

def rpm_to_rad_s(rpm):
	return rpm * 2 * np.pi / 60

# for an unkwnon reason, connect/disconnect works only if this is a function
def test_ref_controller():
	# this the reference controller from Cully et al., 2015 (Nature)
	ctrl = [1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0.5, 0.5, 0.25, 0.75, 0.5, 1, 0, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5, 1, 0.5, 0.5, 0.25, 0.25, 0.5, 1, 0, 0.5, 0.25, 0.75, 0.5]
	r_body = 0.1
	params = {
		# Body
		"r_body": r_body,
		"z_body": 0.04,
		"m_body": 1.031,
		# Servo 1
		"torque_servo_1": 0.8,
		"lower_servo_1": -np.pi/2,
		"upper_servo_1": np.pi/2,
		"v_servo_1": rpm_to_rad_s(77.0),
		# Phalanx 1
		"l1": 0.06,
		"r1": 0.02,
		"m1": 0.02,
		# Servo 2
		"torque_servo_2": 0.8,
		"lower_servo_2": -np.pi/4,
		"upper_servo_2": np.pi/4,
		"v_servo_2": rpm_to_rad_s(77.0),
		# Phalanx 2
		"l2": 0.085,
		"r2": 0.02,
		"m2": 0.184,
		# Servo 3
		"torque_servo_3": 0.8,
		"lower_servo_3": -np.pi/4,
		"upper_servo_3": np.pi/4,
		"v_servo_3": rpm_to_rad_s(77.0),
		# Phalanx 3
		"l3": 0.14,
		"r3": 0.025,
		"m3": 0.04,
		# Leg 0
		"phi_body_leg_0": np.pi/3,
		"x_body_leg_0": - r_body * np.cos(np.pi/3),
		"y_body_leg_0": + r_body * np.sin(np.pi/3),
		"z_body_leg_0": 0.0,
		# Leg 1
		"phi_body_leg_1": 0.0,
		"x_body_leg_1": 0,
		"y_body_leg_1": r_body,
		"z_body_leg_1": 0.0,
		# Leg 2
		"phi_body_leg_2": -np.pi/3,
		"x_body_leg_2": + r_body * np.cos(np.pi/3),
		"y_body_leg_2": + r_body * np.sin(np.pi/3),
		"z_body_leg_2": 0.0,
		# Leg 3
		"phi_body_leg_3": np.pi / 3,
		"x_body_leg_3": + r_body * np.cos(np.pi/3),
		"y_body_leg_3": - r_body * np.sin(np.pi/3),
		"z_body_leg_3": 0.0,
		# Leg 4
		"phi_body_leg_4": 0.0,
		"x_body_leg_4": 0.0,
		"y_body_leg_4": - r_body,
		"z_body_leg_4": 0.0,
		# Leg 5
		"phi_body_leg_5": - np.pi / 3,
		"x_body_leg_5": - r_body * np.cos(np.pi/3),
		"y_body_leg_5": - r_body * np.sin(np.pi/3),
		"z_body_leg_5": 0.0,
	}
	m_total = params["m_body"] + 6 * (params["m1"] + params["m2"] + params["m3"])
	print(f"Total mass: {m_total}")
	urf_path = parse_urdf_file(os.path.join(os.path.dirname(__file__), "parametric_hexapod.urdf"), params)
	print(f"URDF file generated at {urf_path}")
	simu = HexapodSimulator(gui=True, urdf=urf_path)
	controller = HexapodController(ctrl)
	for i in range(0, int(30./simu.dt)): # seconds
		simu.handle_key_events()
		simu.step(controller)
		time.sleep(simu.dt)
	print("=>", simu.get_pos()[0])
	simu.destroy()

if __name__ == "__main__":
	# we do 10 simulations to get some statistics (perfs, reproducibility)
	for k in range(0, 1):
		t0 = time.perf_counter()
		test_ref_controller()# this needs to be in a sub-function...
		print(time.perf_counter() - t0, " ms")
	
