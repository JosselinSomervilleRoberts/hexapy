from abc import ABC, abstractmethod
from typing import Union, List, Callable
import numpy as np


ParameterType = Union[float, int, "Vector", np.ndarray]

class Parameter(ABC):
    @property
    @abstractmethod
    def value(self) -> ParameterType:
        pass

    @value.setter
    @abstractmethod
    def value(self, value: ParameterType):
        pass


class ValueParameter(Parameter):
    def __init__(self, value: ParameterType):
        self._value = value

    @property
    def value(self) -> ParameterType:
        return self._value

    @value.setter
    def value(self, value: ParameterType):
        self._value = value

    def __repr__(self):
        return f"ValueParameter({self._value})"


class ParametricParameter(Parameter):
    def __init__(self, params: List[Parameter], compute: Callable):
        self.params = params
        self.compute = compute

    @property
    def value(self) -> ParameterType:
        return self.compute(*[param.value for param in self.params])
    
    @value.setter
    def value(self, value: ParameterType):
        raise ValueError("Cannot set value of a ParametricParameter")
    
    def __repr__(self):
        return f"ParametricParameter({self.params}, {self.compute})"
    
