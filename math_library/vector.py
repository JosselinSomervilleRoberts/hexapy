from typing import Union
import numpy as np
from multipledispatch import dispatch

from math_library.referential import Referential


class Vector:

    # ========== Constructor ==========
    @dispatch(float, float, float, Referential)
    def __init__(self, x: float, y: float, z: float, referential: Referential):
        self.set_pos(x, y, z, referential)

    @dispatch(int, int, int, Referential)
    def __init__(self, x: int, y: int, z: int, referential: Referential):
        self.set_pos(float(x), float(y), float(z), referential)

    @dispatch(np.ndarray, Referential)
    def __init__(self, np3: np.ndarray, referential: Referential):
        self.set_pos(np3, referential)

    # ========== Setters ==========
    @dispatch(float, float, float, Referential)
    def set_pos(self, x: float, y: float, z: float, referential: Referential):
        self.x = x
        self.y = y
        self.z = z
        self.referential = referential

    @dispatch(np.ndarray, Referential)
    def set_pos(self, np3: np.ndarray, referential: Referential):
        if not np3.shape == (3,) and not np3.shape == (4,):
            raise ValueError("Array must have shape (3,) or (4,)")
        self.x, self.y, self.z = np3[:3]
        self.referential = referential

    # ========== Getters ==========
    def np3(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])
    
    def np4(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z, 1])

    def in_ref(self, referential: Referential) -> 'Vector':
        np4_in_ref = self.referential.from_ref(referential) @ self.np4()
        return Vector(np4_in_ref, referential)
    
    # ========== Operators ==========
    def __add__(self, other: Union[np.ndarray, 'Vector']) -> 'Vector':
        if isinstance(other, np.ndarray):
            if not other.shape == (3,):
                raise ValueError("Array must have shape (3,)")
            return Vector(self.x + other[0], self.y + other[1], self.z + other[2], self.referential)
        elif isinstance(other, Vector):
            if self.referential != other.referential:
                raise ValueError("Cannot add Vectors in different referentials")
            return Vector(self.x + other.x, self.y + other.y, self.z + other.z, self.referential)
        else:
            raise ValueError(f"Unsupported type for addition: {type(other)}")
        
    def __sub__(self, other: Union[np.ndarray, 'Vector']) -> 'Vector':
        if isinstance(other, Vector):
            if self.referential != other.referential:
                raise ValueError("Cannot subtract Vectors in different referentials")
            return Vector(self.x - other.x, self.y - other.y, self.z - other.z, self.referential)
        else:
            return self.__add__(-other)
        
    def __mul__(self, scalar: float) -> 'Vector':
        return Vector(self.x * scalar, self.y * scalar, self.z * scalar, self.referential)
    
    def __repr__(self):
        return f"Vector({self.x}, {self.y}, {self.z}, {self.referential})"
    
    # ========== Methods ==========
    def distance(self, other: 'Vector') -> float:
        if self.referential != other.referential:
            other_in_self_ref = other.in_ref(self.referential)
        return np.linalg.norm(self.np3() - other_in_self_ref.np3())