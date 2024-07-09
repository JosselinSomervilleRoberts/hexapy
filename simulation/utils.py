import argparse
import numpy as np
from typing import Dict, Tuple, List

from simulation.configs.servo import ServoConfig
from models.hexapod import HexapodModel
from math_library.referential import BaseReferential
from math_library.vector import Vector


DEFAULT_VALUES = {
    "m_body": 1.5,
    "z_body": 0.12,
    "height_body": 0.12,
    "r_body": 0.1,
    "r_legs": 0.275,
    "m1": 0.1,
    "m2": 0.1,
    "m3": 0.025,
    "l1": 0.06,
    "l2": 0.09,
    "l3": 0.15,
    "limit_servo_1": np.pi/2,
    "limit_servo_2": 3*np.pi/4,
    "limit_servo_3": 4*np.pi/5,
    "torque_max": 1.5,
    "max_speed_rpm": 100,
    "kp": 0.16*1.2,
    "kd": 0.4*1.2,
    "step_size": 0.13,
    "phi_step_size": np.pi/4,
    "duration_cycle": 0.3,
}


def parse_args():
    parser = argparse.ArgumentParser(description='Simulate a robot')
    parser.add_argument('--frequency', type=float, help='Frequency of the simulation', default=240)
    parser.add_argument('--control-period', type=float, help='Control period of the simulation', default=10)
    parser.add_argument('--urdf-path', type=str, help='Path to the URDF file', default="urdfs/hexapod_parametric.urdf")
    parser.add_argument('--video', type=str, help='Path to the video file', default='')
    parser.add_argument('--disable-gui', action='store_true', help='Disable the GUI')
    parser.add_argument('--disable-safety-turnover', action='store_true', help='Disable the safety turnover')
    for key, value in DEFAULT_VALUES.items():
        parser.add_argument(f"--{key.replace('_', '-')}", type=type(value), default=value)
    return parser.parse_args()

def rpm_to_rad_s(rpm):
	return rpm * 2 * np.pi / 60


def get_params_for_urdf(params: Dict[str, float]) -> Tuple[Dict[str, float], List[ServoConfig]]:
    params_urf = {
        # Body
        "r_body": params["r_body"],
        "z_body": 0.04,
        "m_body": params["m_body"],
        # Servo 1
        "torque_servo_1": params["torque_max"],
        "lower_servo_1": -params["limit_servo_1"],
        "upper_servo_1": params["limit_servo_1"],
        "v_servo_1": rpm_to_rad_s(params["max_speed_rpm"]),
        # Phalanx 1
        "l1": params["l1"],
        "r1": 0.02,
        "m1": params["m1"],
        # Servo 2
        "torque_servo_2": params["torque_max"],
        "lower_servo_2": -params["limit_servo_2"],
        "upper_servo_2": params["limit_servo_2"],
        "v_servo_2": rpm_to_rad_s(params["max_speed_rpm"]),
        # Phalanx 2
        "l2": params["l2"],
        "r2": 0.02,
        "m2": params["m2"],
        # Servo 3
        "torque_servo_3": params["torque_max"],
        "lower_servo_3": -params["limit_servo_3"],
        "upper_servo_3": params["limit_servo_3"],
        "v_servo_3": rpm_to_rad_s(params["max_speed_rpm"]),
        # Phalanx 3
        "l3": params["l3"],
        "r3": 0.02,
        "m3": params["m3"],
        # Leg 0
        "phi_body_leg_0": np.pi/3,
        "x_body_leg_0": - params["r_body"] * np.cos(np.pi/3),
        "y_body_leg_0": + params["r_body"] * np.sin(np.pi/3),
        "z_body_leg_0": 0.0,
        # Leg 1
        "phi_body_leg_1": 0.0,
        "x_body_leg_1": 0,
        "y_body_leg_1": params["r_body"],
        "z_body_leg_1": 0.0,
        # Leg 2
        "phi_body_leg_2": -np.pi/3,
        "x_body_leg_2": + params["r_body"] * np.cos(np.pi/3),
        "y_body_leg_2": + params["r_body"] * np.sin(np.pi/3),
        "z_body_leg_2": 0.0,
        # Leg 3
        "phi_body_leg_3": np.pi / 3,
        "x_body_leg_3": + params["r_body"] * np.cos(np.pi/3),
        "y_body_leg_3": - params["r_body"] * np.sin(np.pi/3),
        "z_body_leg_3": 0.0,
        # Leg 4
        "phi_body_leg_4": 0.0,
        "x_body_leg_4": 0.0,
        "y_body_leg_4": - params["r_body"],
        "z_body_leg_4": 0.0,
        # Leg 5
        "phi_body_leg_5": - np.pi / 3,
        "x_body_leg_5": - params["r_body"] * np.cos(np.pi/3),
        "y_body_leg_5": - params["r_body"] * np.sin(np.pi/3),
        "z_body_leg_5": 0.0,
    }
    servo_configs = [
         ServoConfig(
            kp=params["kp"],
            kd=params["kd"],
            max_torque=params["torque_max"],
            max_velocity=rpm_to_rad_s(params["max_speed_rpm"]),
            lower_limit=-params["limit_servo_1"],
            upper_limit=params["limit_servo_1"]
        ),
        ServoConfig(
            kp=params["kp"],
            kd=params["kd"],
            max_torque=params["torque_max"],
            max_velocity=rpm_to_rad_s(params["max_speed_rpm"]),
            lower_limit=-params["limit_servo_2"],
            upper_limit=params["limit_servo_2"]
        ),
        ServoConfig(
            kp=params["kp"],
            kd=params["kd"],
            max_torque=params["torque_max"],
            max_velocity=rpm_to_rad_s(params["max_speed_rpm"]),
            lower_limit=-params["limit_servo_3"],
            upper_limit=params["limit_servo_3"]
        )
    ] * 6
    return params_urf, servo_configs


def setup_model(params: Dict[str, float], base: BaseReferential) -> HexapodModel:
    leg_start_pos = np.zeros((6, 3))
    desired_pos = np.zeros((6, 3))
    leg_start_angles = np.array([np.pi/3, 0, -np.pi/3, -2*np.pi/3, np.pi, 2*np.pi/3])
    for i in range(6):
        angle = leg_start_angles[i]
        leg_start_pos[i] = np.array([params["r_body"] * np.cos(angle), params["r_body"] * np.sin(angle), 0])
        desired_pos[i] = np.array([params["r_legs"] * np.cos(angle), params["r_legs"] * np.sin(angle), 0])
    hexapod = HexapodModel(base, leg_start_pos, leg_start_angles, [params["l1"], params["l2"], params["l3"]])
    hexapod.set_pos(np.array([0, 0, params["height_body"]]))
    for i in range(6):
        hexapod.legs[i].set_end_pos(Vector(desired_pos[i], base))
    return hexapod