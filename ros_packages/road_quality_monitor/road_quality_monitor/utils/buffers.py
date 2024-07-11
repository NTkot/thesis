import numpy as np
from builtin_interfaces.msg import Time

# Function that returns timestamp in seconds (with decimals) by reading timestamp from Header ROS msg
def get_stamp_time_sec(msg_stamp: Time):
    return msg_stamp.sec + 0.000001*(msg_stamp.nanosec // 1000)

# Class that provides an abstraction-layer for a circular buffer data structure
class circular_buffer:
    def __init__(self, size : int = 50, dtype = int):
        self.data  = np.zeros(size).astype(dtype)   # Array holding data
        self.dtype = dtype                          # data type
        self.size  = size                           # size of array

    # Insert a new element to buffer
    def insert(self, new_element):
        temp          = self.data[-1]           # Get last element about to be tossed out of buffer
        self.data     = np.roll(self.data, -1)  # Move all elements one position to the left
        self.data[-1] = self.dtype(new_element) # Make last element be the new_element
        return temp                             # Return tossed-out element

    # Clears buffer
    def clear(self):
        self.data = np.zeros(self.size).astype(self.dtype)

# Class that provides an abstraction-layer for a moving-average filter
class moving_average:
    def __init__(self, window_length : int = 10):
        self.value         = 0.0            # Output value
        self.window_length = window_length  # Moving average window length
        self.buffer        = circular_buffer(size = window_length, dtype = float)   # Array holding last 'size' values

    # Insert a new element to the buffer and update output value
    def insert(self, new_value):
        self.value = self.value + (new_value / self.window_length) - (self.buffer.data[0] / self.window_length)
        self.buffer.insert(new_value)

# Class that provides an abstraction-layer for timeseries 
class timeseries_buffer:
    def __init__(self, max_size : int = 50, dtype = int):
        self.data     = circular_buffer(size = max_size, dtype = dtype)     # Buffer holding values of timeseries
        self.timedata = circular_buffer(size = max_size, dtype = float)     # Buffer holding timestamps of values
        self.max_size = max_size                                            # Max elements that can be stored
        self.size     = 0                                                   # Current number of elements stored

    # Insert a new pair of values (time, value) into the timeseries
    def insert(self, new_timedata_value, new_data_value):
        self.data.insert(new_data_value)                # Insert value into data buffer
        self.timedata.insert(new_timedata_value)        # Insert time into timedata buffer
        self.size = (self.size + 1) if (self.size < self.max_size) else self.max_size   # Increment size by 1 (or clip it to max_size if size >= max_size)

    # Returns only data elements that correspond to the current number of elements stored
    def active_data(self):
        return self.data.data[-self.size:]

    # Returns only timedata elements that correspond to the current number of elements stored
    def active_timedata(self):
        return self.timedata.data[-self.size:]
    
    # Make everything zero
    def clear(self):
        self.data.clear()
        self.timedata.clear()
        self.size = 0

class rate():
    def __init__(self, filter_size: int):
        self.filter = moving_average(window_length=filter_size)

        self.last_msg_time_sec = get_stamp_time_sec(Time())

    def insert_stamp(self, stamp: Time):
        time_sec = get_stamp_time_sec(stamp)

        freq = 1 / (time_sec - self.last_msg_time_sec)
        self.last_msg_time_sec = time_sec

        self.filter.insert(freq)

    @property
    def value(self):
        return self.filter.value

# Function that checks if last two timedata of a timeseries_buffer are within autoclear_ms range. 
# If they are not, the timeseries is cleared 
def autoclear_checker(timeseries : timeseries_buffer, autoclear_ms):
    x = timeseries.active_timedata()
    if((x.size > 1) and (abs(x[-1] - x[-2]) > (autoclear_ms/1000))):
        timeseries.clear()