from models.hexapod import HexapodModel
from math_library.vector import Vector
from math_library.referential import BaseReferential
import numpy as np

from simulation.simulator import HexapodSimulator, SimulationConfig, SimulationState, HexapodConfig
import os
from simulation.urdf_parser import parse_urdf_file
import time
from function_profiles import _control_signal_bell, interpolate_value_in_array
from simulation.interactions import KeyboardHandler
from simulation.utils import parse_args, get_params_for_urdf, setup_model

HEIGHT_LEG = 0.04


def sign(value: float) -> int:
    return 1 if value >= 0 else -1

class WalkingController:
    def __init__(self, hexapod: HexapodModel, params: dict):
        self.hexapod = hexapod
        self.start_time = None
        self.last_update = 0

        # Params
        self.step_size = params["step_size"]
        self.phi_step_size = params["phi_step_size"]
        self.duration_cycle = params["duration_cycle"]
        self.r_legs = params["r_legs"]
        self.height_body = params["height_body"]
        
        # Values for the walking gait
        self.direction = 0 # angle in rad
        self.speed = 0.3 # m/s
        self.phi_speed = 0. # rad/s

        # Values for the current cycle
        self.cycle_start_time = None
        self.cycle_start_pos = np.zeros(3)
        self.cycle_leg_start_pos = np.zeros((6, 3))
        self.cycle_start_phi = 0
        self.in_cycle = False
        self.cycle_direction = 0.
        self.cycle_speed = 0.
        self.cycle_phi_speed = 0.
        self.last_update = None

    def set_command(self, direction: float, speed: float, phi_speed: float):
        self.direction = direction
        self.speed = speed
        self.phi_speed = phi_speed

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
                self.cycle_end_time = self.cycle_start_time + self.duration_cycle * max(abs(self.speed), abs(self.phi_speed))
                direction = self.direction + hexapod.phi_orientation
                self.cycle_end_pos = self.cycle_start_pos + Vector(self.speed * self.step_size * np.array([np.cos(direction), np.sin(direction), 0]), self.hexapod.base_referential).np3()
                self.cycle_end_phi = self.cycle_start_phi + self.phi_speed * self.phi_step_size
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
            phase = (t - self.cycle_start_time) / self.duration_cycle
            self.angles = self._get_angles_in_cycle(phase)
            return self.angles

            
    def _get_angles_rest(self):
        # At rest the legs are rotated by pi/3 in the pseudo referential
        self.hexapod.z = self.height_body
        angles = []
        positions = []
        angles_ref_leg = np.array([np.pi/3, 0, -np.pi/3, -2*np.pi/3, np.pi, 2*np.pi/3])
        for i in range(6):
            position = Vector(self.r_legs * np.cos(angles_ref_leg[i]), self.r_legs * np.sin(angles_ref_leg[i]), 0., self.hexapod.body_pseudo_referential)
            self.hexapod.legs[i].set_end_pos(position)
            position_np3 = self.hexapod.legs[i].get_positions()[-1].in_ref(self.hexapod.base_referential).np3()
            angles += self.hexapod.legs[i].get_angles()
            positions.append(position_np3)
        #return [0.] * 18, positions
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
        for i in indices_in_air:
            leg_pos[i] = (1 - coef) * self.cycle_leg_start_pos[i] + coef * self.cycle_leg_end_pos[i]
            leg_pos[i][2] = height_leg

        angles = []
        for i in range(6):
            self.hexapod.legs[i].set_end_pos(Vector(leg_pos[i], self.hexapod.base_referential))
            angles += self.hexapod.legs[i].get_angles()
        return angles
    
    def _get_height_for_phase(self, phase: float):
        if not hasattr(self, "height_leg_profile") or self.height_leg_profile is None:
            self.height_leg_profile = _control_signal_bell(0.4, 100) * HEIGHT_LEG
        return interpolate_value_in_array(self.height_leg_profile, phase)
    
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
    def __init__(self, hexapod: HexapodModel):
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
    args = parse_args()
    params_urdf, servo_configs = get_params_for_urdf(vars(args))
    params = {**params_urdf, **vars(args)}
    urf_path = parse_urdf_file(args.urdf_path, params_urdf)
    print(f"URDF file generated at {urf_path}")
    base = BaseReferential()
    hexapod = setup_model(params, base)
    controller = WalkingController(hexapod, params)
    angles = controller._get_angles_rest()[0]
    m_total = params["m_body"] + 6 * (params["m1"] + params["m2"] + params["m3"])
    print(f"Total mass: {m_total}")

    # Simulation
    simu_config = SimulationConfig(
        dt=1./params["frequency"],
        control_period=params["control_period"],
        video=params["video"],
        gui=not params["disable_gui"],
        safety_turnover=not params["disable_safety_turnover"],
    )
    hexapod_config = HexapodConfig(
        urdf=urf_path,
        model=hexapod,
        servo_configs=servo_configs,
    )
    simu = HexapodSimulator(hexapod_config=hexapod_config, config=simu_config)
    keyboard_handler = KeyboardHandler()
    keyboard_handler.physics_client = simu.physics
    last_sleep = time.time()
    error: bool = False
    try:
        while not error:
            keyboard_handler.update(controller)
            error: bool = simu.step(controller)
            state: SimulationState = simu.get_state()
    

            time_to_sleep = simu_config.dt - (time.time() - last_sleep)
            if time_to_sleep > 0:
                time.sleep(time_to_sleep)
            last_sleep = time.time()
        if error:
            print("Error in simulation: safety turnover")
    except KeyboardInterrupt:
        print("Simulation stopped by user")

    simu.destroy()