from __future__ import annotations
import abc
import numpy as np
from copy import copy, deepcopy
from typing import Callable
from typing_extensions import Self
from .labels import Annotation, AnomalyLabel
from .utils import get_by_path, set_by_path, find_objects_vars


class BaseSignalContainer(abc.ABC):
    def __init__(self, *args, **kwargs):
        # Check if 'object' was given as a single argument. Initialize all signals based on the same-type object 
        if (len(args) == 1 and len(kwargs.keys()) == 0 and type(args[0]) == self.__class__) or \
           (len(args) == 0 and len(kwargs.keys()) == 1 and 'object' in kwargs.keys() and type(kwargs['object']) == self.__class__):
                obj = args[0] if len(args) > 0 else kwargs['object']
                for var in self.variables:
                    self.__setattr__(var, deepcopy(obj.__getitem__(var)))

        # Check if 'size' was given as a single argument. Zero-initialize all signals in that case
        elif (len(args) == 1 and len(kwargs.keys()) == 0) or \
             (len(args) == 0 and len(kwargs.keys()) == 1 and 'size' in kwargs.keys()):
            size = args[0] if len(args) > 0 else kwargs['size']
            self.zero_init(size)

        # Else, check if passed arguments correspond to attributes defined in class
        else:
            for i in range(len(args)):
                if self.variables[i] in kwargs.keys():
                    raise ValueError(f"Invalid arguments passed to '{self.__class__.__name__}.__init__()'\n" + 
                                     f"You passed argument '{self.variables[i]}' in both positional and keywords constructor arguments")
                kwargs[self.variables[i]] = args[i]

            if len(kwargs.keys()) == len(self.variables) and all(key in self.variables for key in kwargs.keys()):
                for var in kwargs.keys():
                    self.__setattr__(var, kwargs[var])
            else:
                raise ValueError(f"Invalid arguments passed to '{self.__class__.__name__}.__init__()'\n" + 
                                 f"You need to either pass the following positional/keyword arguments: {self.variables}\n" + 
                                  "or pass a single int argument containing size for zero-initialization")

    @abc.abstractmethod
    def __getitem__(self, item: str):
        return self.__dict__[item]
    
    @abc.abstractmethod
    def __len__(self):
        pass

    @property
    def variables(self) -> list[str]:
        return list([key for key in self.__annotations__.keys() if not key.startswith('_')])
    
    @abc.abstractmethod
    def zero_init(self, size: int):
        pass


class SignalContainer(BaseSignalContainer):
    def __getitem__(self, item: str) -> np.ndarray:
        return super().__getitem__(item)
    
    def __len__(self) -> int:
        return len(self.__getitem__(self.variables[0]))
    
    def __str__(self) -> str:
        ret = ""
        if len(self) > 10:
            for var_name in self.variables:
                var = self.__getitem__(var_name)
                ret += f"{var_name}: [{np.array2string(var[:5], precision=3, separator=', ')[1:-1]}, "
                ret += f"..., "
                ret += f"{np.array2string(var[5:], precision=3, separator=', ')[1:-1]}]\n"
        else:
            for var_name in self.variables:
                var = self.__getitem__(var_name)
                ret += f"{var_name}: {np.array2string(var, precision=3, separator=', ')}\n"
        return ret.rstrip()

    def zero_init(self, size: int):
        for var in self.variables:
            self.__setattr__(var, np.zeros(size))

    def slice(self, idx_start: int, idx_end: int) -> Self:
        kwargs = {}
        for var in self.variables:
            kwargs[var] = copy(self.__getitem__(var)[idx_start:idx_end])
        return self.__class__(**kwargs)


class Vector3(SignalContainer):
    x: np.ndarray
    y: np.ndarray
    z: np.ndarray

class Gps(SignalContainer):
    lon: np.ndarray     # In degrees
    lat: np.ndarray     # In degrees
    speed: np.ndarray   # In km/h

class Time(SignalContainer):
    rel: np.ndarray     # In seconds
    abs: np.ndarray     # In epoch nanoseconds

class Segment(BaseSignalContainer):
    accel: Vector3
    gyro:  Vector3
    gps:   Gps
    time:  Time

    def __getitem__(self, item: str) ->  Vector3 | Gps | Time:
        return super().__getitem__(item)
    
    def __len__(self) -> int:
        return len(self.time)

    def zero_init(self, size: int):
        self.accel = Vector3(size)
        self.gyro  = Vector3(size)
        self.gps   = Gps(size)
        self.time  = Time(size)

    def __str__(self) -> str:
        ret = ""
        for var_name in self.variables:
            ret += f"{var_name}:"
            var = self.__getitem__(var_name)
            ret += '  '.join(('\n' + str(var).lstrip()).splitlines(True))
            ret += '\n'
        return ret.rstrip()

    @property
    def time_vars(self) -> list[str]:
        base_vars = [key for key in self.variables if 'time' in key.lower()]

        out_list = []
        for base_v in base_vars:
            base_path = [base_v]
            out_list.extend(find_objects_vars(get_by_path(self, [base_v]), base_path))

        return out_list
    
    @property
    def vars(self) -> list[list[str]]:
        base_vars = [key for key in self.variables if not 'time' in key.lower()]

        out_list = []
        for base_v in base_vars:
            base_path = [base_v]
            out_list.extend(find_objects_vars(get_by_path(self, [base_v]), base_path))

        return out_list
    
    def slice(self, idx_start: int, idx_end: int) -> Segment:
        return Segment(accel=self.accel.slice(idx_start, idx_end),
                       gyro=self.gyro.slice(idx_start, idx_end),
                       gps=self.gps.slice(idx_start, idx_end),
                       time=self.time.slice(idx_start, idx_end))
    
    def segment(self, samples_per_segment: int, overlap_ratio: float = 0.0) -> list[Segment]:
        offset = int(round(overlap_ratio * samples_per_segment))
        start_indices = [i for i in range(0, len(self) - samples_per_segment + 1, samples_per_segment - offset)]
        end_indices = [elem + offset for elem in start_indices[1:]]
        end_indices.append(start_indices[-1] + samples_per_segment)

        return [self.slice(start, end) for start, end in zip(start_indices, end_indices)]
    
    def transform(self, transform_fun: Callable[[np.ndarray], np.ndarray], vars_whitelist: list[list[str]] = None, in_place: bool = False) -> None | Segment:
        all_vars = self.vars
        if vars_whitelist is not None:
            vars_to_transform = []
            for var in vars_whitelist:
                if not var in all_vars:
                    raise ValueError(f"Variable {var} is not a valid variable in a 'Segment' object")
                vars_to_transform.append(var)
        else:
            vars_to_transform = all_vars

        if in_place:
            for var in vars_to_transform:
                set_by_path(self, var, transform_fun(get_by_path(self, var)))
        else:
            ret = Segment(size=len(self))
            for var in self.vars + self.time_vars:
                if var in vars_to_transform:
                    set_by_path(ret, var, transform_fun(get_by_path(self, var)))
                else:
                    set_by_path(ret, var, get_by_path(self, var))
            return ret

    def _overlap_ratio(self, rel_time_range: list[float]):
        min_rel_time = min(self.time.rel)
        max_rel_time = max(self.time.rel)

        overlap = max(0, min(max_rel_time, rel_time_range[1]) - max(min_rel_time, rel_time_range[0]))
        lengthx = max_rel_time - min_rel_time
        lengthy = rel_time_range[1] - rel_time_range[0]

        # print(f"Overlap: {overlap}s")
        # print(f"Overlap Segment: {100 * overlap / lengthx:.3f}%")
        # print(f"Overlap Annotat: {100 * overlap / lengthy:.3f}%")

        return max(overlap / lengthx, overlap / lengthy)
    
    def is_annotation_in_segment(self, annotation: Annotation, overlap_rules: dict[AnomalyLabel, float] = {}) -> bool:
        overlap_ratio = self._overlap_ratio([annotation.rel_t_start, annotation.rel_t_end])
        # print(f"Overlap: {100 * overlap_ratio:.3f}%")

        if annotation.label.anomaly in overlap_rules.keys():
            overlap_threshold = overlap_rules[annotation.label.anomaly]
        else:
            overlap_threshold = 0.5

        return overlap_ratio >= overlap_threshold
