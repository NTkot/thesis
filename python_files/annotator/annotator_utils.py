import numpy as np
from data_process.labels import Annotation, AnomalyLabel


def saturate(value: [float, int], min_value: [float, int], max_value: [float, int]) -> [float, int]:
    return max(min_value, min(value, max_value))


def binary_search(array, value):
    lo, hi = 0, len(array) - 1
    best_ind = lo
    while lo <= hi:
        mid = lo + (hi - lo) // 2
        if array[mid] < value:
            lo = mid + 1
        elif array[mid] > value:
            hi = mid - 1
        else:
            best_ind = mid
            break
        # check if data[mid] is closer to val than data[best_ind] 
        if abs(array[mid] - value) < abs(array[best_ind] - value):
            best_ind = mid
    return best_ind


class ImuTopicBuffer():
    def __init__(self, abs_time: np.ndarray[int],
                       rel_time: np.ndarray[float],
                       a_x:      np.ndarray[float],
                       a_y:      np.ndarray[float],
                       a_z:      np.ndarray[float],
                       g_x:      np.ndarray[float],
                       g_y:      np.ndarray[float],
                       g_z:      np.ndarray[float]):
        
        if not all([len(abs_time) == len(g_z), len(rel_time) == len(g_z), 
                    len(a_x) == len(g_z), len(a_y) == len(g_z), len(a_z) == len(g_z),
                    len(g_x) == len(g_z), len(g_y) == len(g_z)]):
            raise ValueError('All args on a TopicVector3 constructor must be of same length')
        
        if not all(abs_time[i] <= abs_time[i+1] for i in range(len(abs_time) - 1)):
            raise ValueError('Absolute time vector on TopicVector3 constructor seems NOT to be in ascending order')
        
        if not all(rel_time[i] <= rel_time[i+1] for i in range(len(rel_time) - 1)):
            raise ValueError('Relative time vector on TopicVector3 constructor seems NOT to be in ascending order')
        
        self.abs_time = abs_time
        self.rel_time = rel_time

        self._original_a_x = a_x
        self._original_a_y = a_y
        self._original_a_z = a_z
        self._original_g_x = g_x
        self._original_g_y = g_y
        self._original_g_z = g_z

        self.a_x = a_x
        self.a_y = a_y
        self.a_z = a_z
        self.g_x = g_x
        self.g_y = g_y
        self.g_z = g_z


    def _binary_search_abs_time(self, value):
        return binary_search(self.abs_time, value)
    
    
    def _binary_search_rel_time(self, value):
        return binary_search(self.rel_time, value)


    def abs2rel_time(self, abs_time_value : int) -> float:
        idx = self._binary_search_abs_time(abs_time_value)
        return self.rel_time[idx]
    

    def rel2idx(self, rel_time_value: float) -> int:
        return self._binary_search_rel_time(rel_time_value)


    def apply_filter(self, filter_fun):
        self.a_x = filter_fun(self.a_x)
        self.a_y = filter_fun(self.a_y)
        self.a_z = filter_fun(self.a_z)
        self.g_x = filter_fun(self.g_x)
        self.g_y = filter_fun(self.g_y)
        self.g_z = filter_fun(self.g_z)


    def reset_filters(self):
        self.a_x = self._original_a_x
        self.a_y = self._original_a_y
        self.a_z = self._original_a_z
        self.g_x = self._original_g_x
        self.g_y = self._original_g_y
        self.g_z = self._original_g_z




def annotation_text(annotation: Annotation) -> str:
    length = AnomalyLabel.max_name_len()
    start_str = "{:5.3f}".format(annotation.rel_t_start).rjust(9, ' ')
    end_str   = "{:5.3f}".format(annotation.rel_t_end).rjust(9, ' ')
    return f'{annotation.label.anomaly.name.ljust(length, " ")}: {start_str} - {end_str}'


def annotation_tooltip(annotation: Annotation) -> str:
    return f'{annotation.label.anomaly.name}\n{annotation.label.transversity.name}\n{annotation.label.severity.name}'




# if __name__ == '__main__':
    # a = ImuTopicBuffer(abs_time=np.array([1,2,3,4,5,6]),
    #                    rel_time=np.array([0,1,2,3,4,5]),
    #                    a_x=np.array([5,6,7,8,9,0]),
    #                    a_y=np.array([6,5,7,9,8,0]),
    #                    a_z=np.array([7,6,5,0,8,9]),
    #                    g_x=np.array([5,6,7,8,9,0]),
    #                    g_y=np.array([6,5,7,9,8,0]),
    #                    g_z=np.array([7,6,5,0,8,9]))
    # from data_manip.utils import signal_filter
    # print(a._original_a_x)
    # print(a.a_x)
    # a.apply_filter(lambda x: signal_filter.moving_average(x, 2))
    # print(a._original_a_x)
    # print(a.a_x)
    # a.reset_filters()
    # print(a._original_a_x)
    # print(a.a_x)

    # from data_manip.labels import AnomalyLabel, TransversityLabel, SeverityLabel
    # obj  = Label(AnomalyLabel(3), TransversityLabel(1), SeverityLabel(1))
    # obj2 = Label(AnomalyLabel(1), TransversityLabel(0), SeverityLabel(1))
    # annotations = AnnotationList('helloWorld.file', '2023')
    # annotations.insert(Annotation(0, 1, 0, 1, 0, 1, obj))
    # annotations.insert(Annotation(1, 2, 1, 2, 1, 2, obj2))
