import numpy as np
from typing import List, Dict
from multipledispatch import dispatch

from math_library.vector import Vector
from math_library.referential import ParametricReferential
from math_library.parameter import ValueParameter, ParametricParameter
from math_library.utils import get_rotation_z_transform_matrix, get_rotation_y_transform_matrix, get_translation_transform_matrix


class Leg:

    def __init__(self, leg_start_pos: Vector, lengths: List[float], alpha_0: float = 0):
        self.leg_start_pos = leg_start_pos
        self.body = leg_start_pos.referential
        self.lengths = lengths
        self.alpha_0 = alpha_0
        self._build_leg()

    def _build_leg(self):
        self.parameters = {"alpha": ValueParameter(0), "beta": ValueParameter(0), "gamma": ValueParameter(0)}
        [l1, l2, l3] = self.lengths
        self.referentials = [
            ParametricReferential(
                parent=self.body,
                transformation=ParametricParameter(
                    params=[self.parameters["alpha"]],
                    compute=lambda alpha: get_rotation_z_transform_matrix(alpha, self.leg_start_pos.in_ref(self.body).np3())
                )
            ),
        ]
        self.referentials += [
            ParametricReferential(
                parent=self.referentials[0],
                transformation=ParametricParameter(
                    params=[self.parameters["beta"]],
                    compute=lambda beta: get_rotation_y_transform_matrix(beta, np.array([l1, 0, 0]))
                )
            ),
        ]
        self.referentials += [
            ParametricReferential(
                parent=self.referentials[1],
                transformation=ParametricParameter(
                    params=[self.parameters["gamma"]],
                    compute=lambda gamma: get_rotation_y_transform_matrix(gamma, np.array([l2, 0, 0]))
                )
            )
        ]
        self.referentials += [
            ParametricReferential(
                parent=self.referentials[2],
                transformation=ParametricParameter(
                    params=[],
                    compute=lambda: get_translation_transform_matrix(np.array([l3, 0, 0]))
                )
            )
        ]

    @dispatch(float, float, float)
    def set_angles(self, alpha: float, beta: float, gamma: float):
        self.parameters["alpha"].value = alpha
        self.parameters["beta"].value = beta
        self.parameters["gamma"].value = gamma

    @dispatch(np.ndarray)
    def set_angles(self, angles: np.ndarray):
        if not angles.shape == (3,):
            raise ValueError("Array must have shape (3,)")
        self.set_angles(*angles)

    @property
    def alpha(self) -> float:
        return self.parameters["alpha"].value
    @alpha.setter
    def alpha(self, alpha: float):
        self.parameters["alpha"].value = alpha
    
    @property
    def beta(self) -> float:
        return self.parameters["beta"].value
    @beta.setter
    def beta(self, beta: float):
        self.parameters["beta"].value = beta

    @property
    def gamma(self) -> float:
        return self.parameters["gamma"].value
    @gamma.setter
    def gamma(self, gamma: float):
        self.parameters["gamma"].value = gamma

    def get_angles(self) -> List[float]:
        return [self.alpha, self.beta, self.gamma]
    
    def get_end_pos(self) -> Vector:
        return self.end_pos
    
    def get_positions(self) -> List[Vector]:
        return [Vector(0, 0, 0, ref) for ref in self.referentials]
    
    # Inverse kinematics
    def set_end_pos(self, end_pos: Vector):
        self.end_pos = end_pos
        self._compute_angles()

    def _compute_angles(self):
        A_to_D = self.end_pos.in_ref(self.body) - self.leg_start_pos.in_ref(self.body)
        norm: float = np.linalg.norm(A_to_D.np3())
        if norm > sum(self.lengths):
            raise ValueError(f"Target position is unreachable. Norm: {norm:.3f}, Sum of lengths: {sum(self.lengths):.3f}")
        
        # Alpha
        alpha = np.arctan2(A_to_D.y, A_to_D.x)
        self.alpha = alpha
        # print(f"NORM: {norm}")

        # Beta and gamma
        l1, l2, l3 = self.lengths
        posB = Vector(l1, 0., 0., self.referentials[0])
        B_to_D = self.end_pos.in_ref(self.referentials[0]) - posB.in_ref(self.referentials[0])
        distance: float = np.linalg.norm(B_to_D.np3())
        if distance > l2 + l3:
            raise ValueError(f"Target position is unreachable. Distance: {distance:.3f}, Sum of lengths: {l2 + l3:.3f}")
        cos_gamma = (distance**2 - l2**2 - l3**2) / (2 * l2 * l3)
        gamma = np.arccos(cos_gamma)

        k1 = l2 + l3 * np.cos(gamma)
        k2 = l3 * np.sin(gamma)
        beta = -np.arctan2(B_to_D.z, B_to_D.x) - np.arctan2(k2, k1)
        self.beta = beta
        self.gamma = gamma