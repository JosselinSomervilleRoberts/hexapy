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
from dataclasses import dataclass
import subprocess 
import time
import numpy as np
import pybullet as p
import pybullet_utils.bullet_client as bc
import pybullet_data
from simulation.urdf_parser import parse_urdf_file
from typing import Dict, Optional, List

from pycontrollers.hexapod_controller import HexapodController
from models.hexapod import HexapodModel
from simulation.configs.servo import ServoConfig


MISSING_JOINT = 9999
GRAVITY = -9.81
Z_START = 0.5 # height of the robot at the beginning of the simulation
NUM_INITIAL_STEPS = 100


@dataclass
class SimulationState:
	t: float
	"""Current time in seconds"""

	body_pos: np.ndarray
	"""Position of the body in meters"""

	commands: np.ndarray
	"""Desired position of all servos in radians"""

	angles: np.ndarray
	"""Angles of all servos in radians"""

	velocities: np.ndarray
	"""Velocities of all servos in rad/s"""

	torques: np.ndarray
	"""Torques of all servos in N.m"""

	contact_points: List[int]
	"""List of links in contact with the ground plane"""

	missing_joint_count: int
	"""Number of missing joints (only different from 0 if the robot is broken)"""


@dataclass
class SimulationConfig:
	dt: float
	"""Time in seconds between two steps"""

	control_period: int
	"""Number of steps between two calls of the controller"""

	video: str
	"""Path to the video file to save. If empty, no video is saved"""

	gui: bool
	"""If true, the GUI is displayed"""

	safety_turnover: bool
	"""If true, the robot is turned over if it falls"""


@dataclass
class HexapodConfig:
	urdf: str
	"""Path to the URDF file of the robot"""

	model: HexapodModel
	"""Model of the robot"""

	servo_configs: Dict[int, ServoConfig]
	"""Configuration of all servos (indexed by their ID)"""


class HexapodSimulator:
	def __init__(self, hexapod_config: HexapodConfig, config: SimulationConfig):
		self.model = hexapod_config.model
		self.hexapod_config = hexapod_config
		self.config = config
		self.t = 0
		self.i = 0
		self.last_contact_time: Dict[int, float] = {}

		# the desired position for the joints
		self.commands = np.array(self.model.get_servo_angles())
		self.last_step_time = time.time()
		self.avg_fps = 0.
		self._create_physics_client()
		self.planeId = self.physics.loadURDF("plane.urdf")
		self._load_robot_urdf()

		# bullet links number corresponding to the legs
		self.leg_link_ids = [17, 14, 2, 5, 8, 11]
		self.descriptor = {17 : [], 14 : [], 2 : [], 5 : [], 8 : [], 11 : []}

		# Friction [DISABLED FOR NOW]
		# self.set_friction(self.planeId, lateral_friction=1.0)  # Set the desired friction coefficient
		# self.set_robot_feet_friction(self.botId, lateral_friction=1.0)  # Set the desired friction coefficient

		# video makes things much slower
		if (self.config.video != ''):
			self._stream_to_ffmpeg(self.config.video)

		# put the hexapod on the ground (gently)
		self.physics.setRealTimeSimulation(0)
		self._run_initial_steps()

	def _setup_gui(self):
		assert self.config.gui
		self.physics.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
		self.physics.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
		self.physics.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
		self.physics.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
		self.physics.resetDebugVisualizerCamera(cameraDistance=1,
												cameraYaw=20,
												cameraPitch=-20,
												cameraTargetPosition=[1, -0.5, 0.8])

	def _create_physics_client(self):
		if self.config.gui:
			self.physics = bc.BulletClient(connection_mode=p.GUI)
			self._setup_gui()
		else:
			self.physics = bc.BulletClient(connection_mode=p.DIRECT)

		self.physics.setAdditionalSearchPath(pybullet_data.getDataPath())
		self.physics.resetSimulation()
		self.physics.setGravity(0,0,GRAVITY)
		self.physics.setTimeStep(self.config.dt)
		self.physics.setPhysicsEngineParameter(fixedTimeStep=self.config.dt)

	def _load_robot_urdf(self):
		start_pos = self.model.get_pos() + np.array([0, 0, Z_START])
		start_orientation = self.physics.getQuaternionFromEuler([0.,0,0])
		self.botId = self.physics.loadURDF(self.hexapod_config.urdf, basePosition=start_pos, baseOrientation=start_orientation, flags=(p.URDF_USE_SELF_COLLISION))
		self.joint_list = self._make_joint_list(self.botId)

	def _run_initial_steps(self):
		self.commands = self.model.get_servo_angles()
		for t in range(0, NUM_INITIAL_STEPS):
			self._move_joints()
			self.physics.setGravity(0,0, GRAVITY)
			self.physics.stepSimulation()

	def set_friction(self, body_id, lateral_friction):
		self.physics.changeDynamics(body_id, -1, lateralFriction=lateral_friction)

	def set_robot_feet_friction(self, robot_id, lateral_friction):
		for leg_link_id in self.leg_link_ids:
			self.physics.changeDynamics(robot_id, leg_link_id, lateralFriction=lateral_friction)


	def get_state(self) -> SimulationState:
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

		return SimulationState(
			t = self.t,
			body_pos = self.get_pos(),
			commands = self.commands,
			angles = self.joint_angles,
			velocities = self.joint_velocities,
			torques = self.joint_torques,
			contact_points = self.get_contact_points(),
			missing_joint_count = self.missing_joint_count
		)

	def destroy(self):
		try:
			self.physics.disconnect()
			if self.config.video != '':
				self.ffmpeg_pipe.stdin.close()
				self.ffmpeg_pipe.stderr.close()
				self.ffmpeg_pipe.wait()
		except p.error as e:
			print("Warning (destructor of simulator):", e)

	def _update_contact_points(self):
		# 1. Get contact points between robot and world plane 
		# at the current time step
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

		# 2. Update the last contact time for each link
		for l in link_ids:
			self.last_contact_time[l] = self.t

	def get_contact_points(self, last_x_dt: Optional[int] = None, auto_delete: bool = True):
		if last_x_dt is None:
			last_x_dt = max(1, self.config.control_period // 2)
		contact_points = []
		for l in self.last_contact_time.copy():
			if self.t - self.last_contact_time[l] < last_x_dt * self.config.dt:
				contact_points.append(l)
			elif auto_delete:
				del self.last_contact_time[l]
		return contact_points

	def reset(self):
		assert(0), "not working for now"
		self.t = 0
		self.physics.resetSimulation()

	def get_pos(self):
		'''
		Returns the position list of 3 floats and orientation as list of 4 floats in [x,y,z,w] order.
		Use p.getEulerFromQuaternion to convert the quaternion to Euler if needed.
		'''
		return self.physics.getBasePositionAndOrientation(self.botId)
	
	def _save_video_if_necessary(self):
		# 24 FPS
		if self.config.video != '' and self.i % (int(1. / (self.dt * 24))) == 0: 
			camera = self.physics.getDebugVisualizerCamera()
			img = p.getCameraImage(camera[0], camera[1], renderer=p.ER_BULLET_HARDWARE_OPENGL)
			self.ffmpeg_pipe.stdin.write(img[2].tobytes())

	def _check_safety_turnover(self) -> bool:
		if not self.config.safety_turnover:
			return False
		#Check if roll pitch are not too high
		self.euler = self.physics.getEulerFromQuaternion(self.get_pos()[1])
		if((abs(self.euler[1]) >= math.pi/2) or (abs(self.euler[0]) >= math.pi/2)):
			return True
		return False
	
	def _move_joints(self):
		self.missing_joint_count = 0
		for j, joint_id in enumerate(self.joint_list):
			if(joint_id==MISSING_JOINT):
				self.missing_joint_count += 1
			else:
				# info = self.physics.getJointInfo(self.botId, joint_id)
				servo_config: ServoConfig = self.hexapod_config.servo_configs[joint_id]
				pos = min(max(servo_config.lower_limit, self.commands[j]), servo_config.upper_limit)
				self.physics.setJointMotorControl2(self.botId, joint_id,
					p.POSITION_CONTROL,
					positionGain=servo_config.kp,
					velocityGain=servo_config.kd,
					targetPosition=pos,
					force=servo_config.max_torque,
					maxVelocity=servo_config.max_velocity
				)

	def step(self, controller):
		if self.i % self.config.control_period == 0:
			self.commands = controller.step(self)
		self.i += 1
		
		self._save_video_if_necessary()
		error = self._check_safety_turnover()
		self._move_joints()
		self.physics.setGravity(0, 0, GRAVITY)
		self.physics.stepSimulation()
		self.t += self.config.dt
		self._update_contact_points()
		return error


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
				joint_list += [MISSING_JOINT] #if the joint is not here (aka broken leg case) put MISSING_JOINT
		return joint_list


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
	
