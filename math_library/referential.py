from abc import ABC
import numpy as np

from math_library.parameter import ParametricParameter


class Referential(ABC):
    @staticmethod
    def from_ref(self, referential: 'Referential') -> np.ndarray:
        pass

class BaseReferential(Referential):
    def from_ref(self, referential: Referential) -> np.ndarray:
        if self == referential:
            return np.eye(4)
        transform_inversed = referential.from_ref(self)
        return np.linalg.inv(transform_inversed)


class ParametricReferential(Referential):

    def __init__(self, parent: Referential, transformation: ParametricParameter):
        self.parent: Referential = parent
        self.transformation: ParametricParameter = transformation

    def from_ref(self, referential: Referential) -> np.ndarray:
        # From itself, the transformation matrix is the identity matrix
        if self == referential:
            return np.eye(4)
        transformation_matrix = self.transformation.value
        return  self.parent.from_ref(referential) @ transformation_matrix
