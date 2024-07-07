from hexapod import Hexapod
from math_library.vector import Vector
from math_library.referential import BaseReferential
import numpy as np

from simulator import HexapodSimulator, rpm_to_rad_s
import os
from urdf_parser import parse_urdf_file
import time


BODY_HEIGHT = 0.08
R_LEGS = 0.3

class WalkingController:
    def __init__(self, hexapod: Hexapod):
        self.hexapod = hexapod
        self.start_time = None
        self.last_update = 0
        
        # Values for the walking gait
        self.direction = np.pi/2 # angle in rad
        self.speed = 0.3 # m/s
        self.phi_speed = 0. # rad/s

        # Values for the current cycle
        self.cycle_duration = 0.5 # s
        self.cycle_start_time = None
        self.cycle_start_pos = np.zeros(3)
        self.cycle_leg_start_pos = np.zeros((6, 3))
        self.cycle_start_phi = 0
        self.in_cycle = False
        self.cycle_direction = 1
        self.cycle_speed = 0.1
        self.cycle_phi_speed = 0.0
        self.last_update = None

    def step(self, simulator):
        t = simulator.t
        if self.last_update is None:
            self.last_update = t
        WARMPUP_TIME = 2
        if t < WARMPUP_TIME:
            return self._get_angles_rest()[0]
        t -= WARMPUP_TIME

        # End of the cycle
        if self.in_cycle and t >= self.cycle_end_time:
            # Finish the cycle
            self.in_cycle = False
            self.hexapod.set_pos(self.cycle_end_pos)
            self.hexapod.phi_orientation = self.cycle_end_phi
            angles, pos = self._get_angles_rest()
            for i in range(6):
                self.hexapod.legs[i].set_end_pos(Vector(pos[i], self.hexapod.base_referential))
            t_start = self.cycle_end_time
        elif not self.in_cycle:
            t_start = self.last_update
        self.last_update = t

        # Start a new cycle
        if not self.in_cycle:
            if abs(self.speed) > 0 or abs(self.phi_speed) > 0:
                # Start position
                self.cycle_start_time = t_start
                self.cycle_start_pos = self.hexapod.get_pos()
                self.cycle_leg_start_pos = self._get_angles_rest()[1]
                self.cycle_start_phi = self.hexapod.phi_orientation

                # Destination
                self.cycle_end_time = self.cycle_start_time + self.cycle_duration
                self.cycle_end_pos = self.cycle_start_pos + Vector(self.speed * self.cycle_duration * np.array([np.cos(self.direction), np.sin(self.direction), 0]), self.hexapod.base_referential).np3()
                self.cycle_end_phi = self.cycle_start_phi + self.phi_speed * self.cycle_duration
                self.hexapod.phi_orientation = self.cycle_end_phi
                self.hexapod.set_pos(self.cycle_end_pos)
                self.cycle_leg_end_pos = self._get_angles_rest()[1]
                self.hexapod.set_pos(self.cycle_start_pos)
                self.hexapod.phi_orientation = self.cycle_start_phi

                self.in_cycle = True
                self.cycle_direction = self.direction
                self.cycle_speed = self.speed
                self.cycle_phi_speed = self.phi_speed
            else:
                # Return current angles
                self.angles = []
                for i in range(6):
                    self.angles += self.hexapod.legs[i].get_angles()
                return self.angles
            
        # In a cycle
        if self.in_cycle:
            phase = (t - self.cycle_start_time) / self.cycle_duration
            self.angles = self._get_angles_in_cycle(phase)
            return self.angles

            
    def _get_angles_rest(self):
        # At rest the legs are rotated by pi/3 in the pseudo referential
        self.hexapod.z = BODY_HEIGHT
        angles = []
        positions = []
        angles_ref_leg = np.array([np.pi/3, 0, -np.pi/3, -2*np.pi/3, np.pi, 2*np.pi/3])
        for i in range(6):
            position = Vector(R_LEGS * np.cos(angles_ref_leg[i]), R_LEGS * np.sin(angles_ref_leg[i]), 0., self.hexapod.body_pseudo_referential)
            self.hexapod.legs[i].set_end_pos(position)
            position_np3 = self.hexapod.legs[i].get_positions()[-1].in_ref(self.hexapod.base_referential).np3()
            angles += self.hexapod.legs[i].get_angles()
            positions.append(position_np3)
        return angles, positions

            
    def _get_angles_in_cycle(self, phase: float):
        # We are in a cycle going at speed self.cycle_speed and rotating at speed self.cycle_phi_speed
        # The phase is in [0, 1]
        # The body moves at the given speed during the entire cycle
        # The legs move in the following way:
        # - 0.5 * cycle_duration: legs 0, 2, 4 are on the ground, legs 1, 3, 5 are in the air
        # - 0.5 * cycle_duration: legs 1, 3, 5 are on the ground, legs 0, 2, 4 are in the air
        # The legs in the air move in a straight line from their current position to their desired position
        assert self.in_cycle

        # Body position
        direction_vector = np.array([np.cos(self.cycle_direction), np.sin(self.cycle_direction), 0])
        self.hexapod.set_pos(self.cycle_start_pos * (1 - phase) + self.cycle_end_pos * phase)
        self.hexapod.phi_orientation = self.cycle_start_phi * (1 - phase) + self.cycle_end_phi * phase

        # Leg positions
        leg_pos = self.cycle_leg_start_pos.copy()
        phase_rel = 2 * phase if phase < 0.5 else 2 * (phase - 0.5)
        if phase < 0.5:
            indices_on_ground = [0, 2, 4]
            indices_in_air = [1, 3, 5]
        else:
            indices_on_ground = [1, 3, 5]
            indices_in_air = [0, 2, 4]
            for i in indices_on_ground:
                leg_pos[i] = self.cycle_leg_end_pos[i]

        height_leg = self._get_height_for_phase(phase_rel)
        coef = self._get_mixing_coefficient(phase_rel)
        print(f"Phase: {phase:.3f}, Height: {height_leg:.3f}, Body height: {self.hexapod.z:.3f}")
        for i in indices_in_air:
            leg_pos[i] = (1 - coef) * self.cycle_leg_start_pos[i] + coef * self.cycle_leg_end_pos[i]
            leg_pos[i][2] = height_leg

        angles = []
        for i in range(6):
            self.hexapod.legs[i].set_end_pos(Vector(leg_pos[i], self.hexapod.base_referential))
            angles += self.hexapod.legs[i].get_angles()
        return angles
    
    def _get_height_for_phase(self, phase: float):
        HEIGHT_LEGS = 0.04
        height_leg = HEIGHT_LEGS * np.sin(np.pi * phase)
        return height_leg
    
    def _get_mixing_coefficient(self, phase: float):
        DEAD_ZONE = 0.1
        if phase <= DEAD_ZONE:
            return 0
        elif phase >= 1 - DEAD_ZONE:
            return 1
        else:
            phase_rel = (phase - DEAD_ZONE) / (1 - 2 * DEAD_ZONE)
            return np.sin(np.pi * phase_rel / 2)


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
    l1, l2, l3 = 0.06, 0.09, 0.15
    lengths = np.array([l1, l2, l3])
    base = BaseReferential()
    leg_start_pos = np.zeros((6, 3))
    desired_pos = np.zeros((6, 3))
    for i in range(6):
        angle = leg_start_angles[i]
        leg_start_pos[i] = np.array([r_body * np.cos(angle), r_body * np.sin(angle), 0])
        desired_pos[i] = np.array([R_LEGS * np.cos(angle), R_LEGS * np.sin(angle), 0])
    print(leg_start_pos)
    hexapod = Hexapod(base, leg_start_pos, leg_start_angles, lengths)
    hexapod.set_pos(np.array([0, 0, BODY_HEIGHT]))
    for i in range(6):
        hexapod.legs[i].set_end_pos(Vector(desired_pos[i], base))
    #for i in range(6):
    #    hexapod.legs[i].set_angles(hexapod.legs[1].alpha, hexapod.legs[1].beta, hexapod.legs[1].gamma)
    for j in range(6):
        print(f"Desired: {desired_pos[j]}, Actual: {hexapod.legs[j].get_positions()[-1].in_ref(base).np3()}, Angles: {hexapod.legs[j].get_angles()}")
    controller = WalkingController(hexapod)
    angles = controller._get_angles_rest()[0]
    for i in range(6):
        hexapod.legs[i].set_angles(np.array(angles[3*i:3*(i+1)]))
    # print(f"Step: {controller.step(None)}")
    TORQUE = 5.0 # Nm
    MAX_SPEED = 2000.0 # rpm
    params = {
        # Body
        "r_body": r_body,
        "z_body": 0.04,
        "m_body": 1.031,
        # Servo 1
        "torque_servo_1": TORQUE,
        "lower_servo_1": -np.pi/2,
        "upper_servo_1": np.pi/2,
        "v_servo_1": rpm_to_rad_s(MAX_SPEED),
        # Phalanx 1
        "l1": l1,
        "r1": 0.02,
        "m1": 0.02,
        # Servo 2
        "torque_servo_2": TORQUE,
        "lower_servo_2": -np.pi,
        "upper_servo_2": np.pi,
        "v_servo_2": rpm_to_rad_s(MAX_SPEED),
        # Phalanx 2
        "l2": l2,
        "r2": 0.02,
        "m2": 0.184,
        # Servo 3
        "torque_servo_3": TORQUE,
        "lower_servo_3": -np.pi,
        "upper_servo_3": np.pi,
        "v_servo_3": rpm_to_rad_s(MAX_SPEED),
        # Phalanx 3
        "l3": l3,
        "r3": 0.02,
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
    simu = HexapodSimulator(hexapod, gui=True, urdf=urf_path, dt=1./30)
    for i in range(0, int(30./simu.dt)): # seconds
        simu.handle_key_events(controller)
        simu.step(controller)
        time.sleep(simu.dt)
    print("=>", simu.get_pos()[0])
    simu.destroy()