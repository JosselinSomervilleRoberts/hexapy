from typing import List, Dict
from multipledispatch import dispatch
import numpy as np

from leg import Leg
from math_library.vector import Vector
from math_library.referential import ParametricReferential, BaseReferential
from math_library.parameter import ValueParameter, ParametricParameter
from math_library.utils import get_rotation_z_transform_matrix, get_rotation_y_transform_matrix, get_rotation_x_transform_matrix


class Hexapod:
    def __init__(self, base_referential: BaseReferential, leg_start_pos: np.ndarray, leg_start_phi: np.ndarray, lengths: List[float]):
        self.base_referential = base_referential
        self._build_body_referential()

        self.legs = []
        self.legs_referentials = []
        for i in range(6):
            referential_start_leg = ParametricReferential(
                parent=self.body_referential,
                transformation=ParametricParameter(
                    params=[],
                    compute=lambda phi=leg_start_phi[i].copy(), pos=leg_start_pos[i].copy(): get_rotation_z_transform_matrix(phi, pos)
                )
            )
            self.legs_referentials.append(referential_start_leg)
            print("Origin of leg 0", Vector(0, 0, 0, self.legs_referentials[0]).in_ref(base_referential).np3())
            leg = Leg(Vector(0, 0, 0, referential_start_leg), lengths, leg_start_phi[i])
            self.legs.append(leg)

    def _build_body_referential(self):
        self.parameters = {
            "pos": ValueParameter(Vector(0., 0., 1., self.base_referential)), "phi_orientation": ValueParameter(0),
            "phi": ValueParameter(0), "psi": ValueParameter(0), "xi": ValueParameter(0)
        }
        self.body_pseudo_referential = ParametricReferential(
                parent=self.base_referential,
                transformation=ParametricParameter(
                    params=[self.parameters["pos"], self.parameters["phi_orientation"]],
                    compute=lambda pos, phi: get_rotation_z_transform_matrix(phi, np.array([pos.x, pos.y, 0.]))
                )
            )
        self.body_referential = ParametricReferential(
                parent=self.body_pseudo_referential,
                transformation=ParametricParameter(
                    params=[self.parameters["pos"], self.parameters["phi"], self.parameters["psi"], self.parameters["xi"]],
                    compute=lambda pos, phi, psi, xi: get_rotation_z_transform_matrix(phi, np.array([0, 0, pos.z])) @ get_rotation_y_transform_matrix(psi, np.array([0, 0, 0])) @ get_rotation_x_transform_matrix(xi, np.array([0, 0, 0]))
                )
            )

    @property
    def x(self) -> float:
        return self.parameters["pos"].value.x
    @x.setter
    def x(self, x: float):
        self.parameters["pos"].value.x = x

    @property
    def y(self) -> float:
        return self.parameters["pos"].value.y
    @y.setter
    def y(self, y: float):
        self.parameters["pos"].value.y = y

    @property
    def z(self) -> float:
        return self.parameters["pos"].value.z
    @z.setter
    def z(self, z: float):
        self.parameters["pos"].value.z = z

    @dispatch(float, float, float)
    def set_pos(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

    @dispatch(np.ndarray)
    def set_pos(self, pos: np.ndarray):
        if not pos.shape == (3,) and not pos.shape == (4,):
            raise ValueError("Array must have shape (3,) or (4,)")
        self.set_pos(*pos[:3])

    def get_pos(self) -> np.ndarray:
        return self.parameters["pos"].value.np3()

    @property
    def phi_orientation(self) -> float:
        return self.parameters["phi_orientation"].value
    @phi_orientation.setter
    def phi_orientation(self, phi_orientation: float):
        self.parameters["phi_orientation"].value = phi_orientation

    @property
    def phi(self) -> float:
        return self.parameters["phi"].value
    @phi.setter
    def phi(self, phi: float):
        self.parameters["phi"].value = phi

    @property
    def psi(self) -> float:
        return self.parameters["psi"].value
    @psi.setter
    def psi(self, psi: float):
        self.parameters["psi"].value = psi

    @property
    def xi(self) -> float:
        return self.parameters["xi"].value
    @xi.setter
    def xi(self, xi: float):
        self.parameters["xi"].value = xi

    @dispatch(float, float, float)
    def set_angles(self, phi: float, psi: float, xi: float):
        self.phi = phi
        self.psi = psi
        self.xi = xi

    @dispatch(np.ndarray)
    def set_angles(self, angles: np.ndarray):
        if not angles.shape == (3,):
            raise ValueError("Array must have shape (3,)")
        self.set_angles(*angles)

    def get_servo_angles(self) -> List[float]:
        angles = []
        for leg in self.legs:
            angles += leg.get_angles()
        return angles
