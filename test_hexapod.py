from hexapod import Hexapod
from math_library.vector import Vector
from math_library.referential import BaseReferential
import numpy as np

from simulator import HexapodSimulator, rpm_to_rad_s
import os
import time
from urdf_parser import parse_urdf_file

class DummyController:
    def __init__(self, hexapod: Hexapod):
        self.hexapod = hexapod
        self.start_time = time.time()

    def step(self, simulator):
        t = time.time() - self.start_time
        self.last_update = t

        R = 0.03
        FREQUENCY = 0.1
        TRANSITION_TIME = 2
        BOOTUP_TIME = 2
        HEIGHT = 0.1
        self.hexapod.set_pos(np.array([0, 0, HEIGHT]))
        heights = np.zeros(6)
        if t < BOOTUP_TIME:
            height = 0
        else:
            self.hexapod.xi = np.pi/12 * np.sin(2 * np.pi * FREQUENCY * (t - BOOTUP_TIME))
            # if (t - BOOTUP_TIME) % (FREQUENCY) < 0.5 * FREQUENCY:
            #     for i in range(0, 6, 2):
            #         heights[i] = HEIGHT * np.abs(np.sin(2 * np.pi * FREQUENCY * (t - BOOTUP_TIME)))
            # else:
            #     for i in range(1, 6, 2):
            #         heights[i] = HEIGHT * np.abs(np.sin(2 * np.pi * FREQUENCY * (t - BOOTUP_TIME)))
            print(f"t: {t}, heights: {heights}")
        # elif t < BOOTUP_TIME + TRANSITION_TIME:
        #     self.hexapod.set_pos(np.array([R * (t - BOOTUP_TIME) / TRANSITION_TIME, 0, HEIGHT]))
        # else:
        #     t -= TRANSITION_TIME + BOOTUP_TIME
        #     self.hexapod.set_pos(np.array([R * np.cos(2 * np.pi * FREQUENCY * t), R * np.sin(2 * np.pi * FREQUENCY * t), HEIGHT]))
        #self.hexapod.set_pos(np.array([0.03 * np.sin(2 * np.pi * t), 0, 0.08 + 0.03 * np.sin(2 * np.pi * t) ]))

        r_legs = 0.28
        desired_pos = np.zeros((6, 3))
        leg_start_angles = np.array([np.pi/3, 0, -np.pi/3, -2*np.pi/3, np.pi, 2*np.pi/3])
        desired_pos = np.zeros((6, 3))
        for i in range(6):
            angle = leg_start_angles[i]
            desired_pos[i] = np.array([r_legs * np.cos(angle), r_legs * np.sin(angle), heights[i]])
        for i in range(6):
            self.hexapod.legs[i].set_end_pos(Vector(desired_pos[i], self.hexapod.base_referential))

        self.angles = []
        for i in range(6):
            angles = self.hexapod.legs[i].get_angles()
            self.angles += angles

        for j in range(6):
            print(f"Desired: {desired_pos[j]}, Actual: {hexapod.legs[j].get_positions()[-1].in_ref(self.hexapod.base_referential).np3()}, Angles: {hexapod.legs[j].get_angles()}")

        return self.angles

if __name__ == "__main__":
    leg_start_angles = np.array([np.pi/3, 0, -np.pi/3, -2*np.pi/3, np.pi, 2*np.pi/3])
    r_body: float = 0.1
    r_legs = 0.32
    lengths = np.array([0.1, 0.1, 0.1])
    base = BaseReferential()
    leg_start_pos = np.zeros((6, 3))
    desired_pos = np.zeros((6, 3))
    for i in range(6):
        angle = leg_start_angles[i]
        leg_start_pos[i] = np.array([r_body * np.cos(angle), r_body * np.sin(angle), 0])
        desired_pos[i] = np.array([r_legs * np.cos(angle), r_legs * np.sin(angle), 0])
    print(leg_start_pos)
    hexapod = Hexapod(base, leg_start_pos, leg_start_angles, lengths)
    hexapod.set_pos(np.array([0, 0, 0.08]))
    for i in range(6):
        hexapod.legs[i].set_end_pos(Vector(desired_pos[i], base))
    #for i in range(6):
    #    hexapod.legs[i].set_angles(hexapod.legs[1].alpha, hexapod.legs[1].beta, hexapod.legs[1].gamma)
    for j in range(6):
        print(f"Desired: {desired_pos[j]}, Actual: {hexapod.legs[j].get_positions()[-1].in_ref(base).np3()}, Angles: {hexapod.legs[j].get_angles()}")
    controller = DummyController(hexapod)
    print(f"Step: {controller.step(None)}")
    TORQUE = 5.0
    params = {
        # Body
        "r_body": r_body,
        "z_body": 0.04,
        "m_body": 1.031,
        # Servo 1
        "torque_servo_1": TORQUE,
        "lower_servo_1": -np.pi/2,
        "upper_servo_1": np.pi/2,
        "v_servo_1": rpm_to_rad_s(77.0),
        # Phalanx 1
        "l1": 0.06,
        "r1": 0.02,
        "m1": 0.02,
        # Servo 2
        "torque_servo_2": TORQUE,
        "lower_servo_2": -np.pi,
        "upper_servo_2": np.pi,
        "v_servo_2": rpm_to_rad_s(77.0),
        # Phalanx 2
        "l2": 0.085,
        "r2": 0.02,
        "m2": 0.184,
        # Servo 3
        "torque_servo_3": TORQUE,
        "lower_servo_3": -np.pi,
        "upper_servo_3": np.pi,
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
    simu = HexapodSimulator(hexapod, gui=True, urdf=urf_path)
    for i in range(0, int(30./simu.dt)): # seconds
        simu.handle_key_events()
        simu.step(controller)
        time.sleep(simu.dt)
    print("=>", simu.get_pos()[0])
    simu.destroy()