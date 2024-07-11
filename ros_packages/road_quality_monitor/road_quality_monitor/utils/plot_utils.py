import numpy as np
from pyqtgraph import PlotDataItem
from buffers import timeseries_buffer

def update_timeplot(plot : PlotDataItem, timeseries : timeseries_buffer):
    if np.all(timeseries.size == 0):
        return
    
    times = timeseries.active_timedata()
    times = times - times[-1]
    plot.setData(times, timeseries.active_data())
