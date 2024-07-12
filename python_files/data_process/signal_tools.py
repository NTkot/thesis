import numpy as np
from scipy import fft
from scipy import signal as sgn
from scipy import interpolate
from statsmodels.tsa import stattools


class signal_manip():
    def _check_time_signal_args(self, time: np.ndarray, 
                                      signal: np.ndarray | list[np.ndarray]):
        if isinstance(signal, list):
            if not all(len(sig) == len(time) for sig in signal):
                raise ValueError("'time' and signals in 'signal' list need to have same length")
        elif len(time) != len(signal):
            raise ValueError("'time' and 'signal' args need to have same length")
        
    @staticmethod
    def find_time_slice_idx(time: np.ndarray, time_range: np.ndarray, range_inside_time_check: bool = False) -> tuple[int, int]:
        if range_inside_time_check and time_range[0] < time[0]:
            raise ValueError(f'find_time_slice_idx(): called with range_inside_time_check flag but lower time range is outside time')
        if range_inside_time_check and time_range[1] > time[-1]:
            raise ValueError(f'find_time_slice_idx(): called with range_inside_time_check flag but upper time range is outside time')

        idx_start = min(range(len(time)), key=lambda i: abs(time[i]-time_range[0]))
        idx_end   = min(range(len(time)), key=lambda i: abs(time[i]-time_range[1]))

        return idx_start, idx_end

    @classmethod
    def slice(cls, time: np.ndarray, 
                   signal: np.ndarray | list[np.ndarray],
                   time_range: np.ndarray = None,
                   idx_range: np.ndarray = None) -> tuple[np.ndarray, np.ndarray] | \
                                                    tuple[np.ndarray, list[np.ndarray]]:
        cls._check_time_signal_args(cls, time, signal)

        if (time_range is None) and (idx_range is None):
            return time, signal
        elif (time_range is None) and (not (idx_range is None)):
            if isinstance(signal, list):
                signal_output = [sig[idx_range[0]:idx_range[1]] for sig in signal]
            else:
                signal_output = signal[idx_range[0]:idx_range[1]]
            time   = time[idx_range[0]:idx_range[1]]
        elif (not (time_range is None)) and (idx_range is None):
            idx_start, idx_end = cls.find_time_slice_idx(time, time_range)
            if isinstance(signal, list):
                signal_output = [sig[idx_start:idx_end] for sig in signal]
            else:
                signal_output = signal[idx_start:idx_end]
            time   = time[idx_start:idx_end]
        elif (not (time_range is None)) and (not (idx_range is None)):
            raise ValueError("You need to specify at most one of the arguments: 'time_range' and 'idx_range'")
        
        return time, signal_output

    @classmethod
    def interp(cls, time: np.ndarray,
                    signal: np.ndarray | list[np.ndarray],
                    time_new: np.ndarray,
                    interp_method = 'linear',
                    **kwargs) -> np.ndarray | list[np.ndarray]:
        cls._check_time_signal_args(cls, time, signal)

        if interp_method == 'linear':
            if isinstance(signal, list):
                return [np.interp(x=time_new, xp=time, fp=sig, **kwargs) for sig in signal]
            else:
                return np.interp(x=time_new, xp=time, fp=signal, **kwargs)

        elif interp_method == 'cubic_spline':
            if isinstance(signal, list):
                out = []
                for sig in signal:
                    spl = interpolate.CubicSpline(x=time, y=sig, **kwargs)
                    out.append(spl(time_new))
                return out
            else:
                spl = interpolate.CubicSpline(x=time, y=signal, **kwargs)
                return spl(time_new)
            
        elif interp_method == 'b_spline':
            if isinstance(signal, list):
                out = []
                for sig in signal:
                    bspl = interpolate.make_interp_spline(x=time, y=sig, **kwargs)
                    out.append(bspl(time_new))
                return out
            else:
                bspl = interpolate.make_interp_spline(x=time, y=signal, **kwargs)
                return bspl(time_new)

        else:
            raise ValueError(f"Unknown interpolation method {interp_method}")


class signal_transform():
    def _fft_signal(signal: np.ndarray, sample_rate: float, window_fun: str = None) -> tuple[np.ndarray, np.ndarray]:
        N = len(signal)
        freqs = fft.fftfreq(N, 1/sample_rate)[:N // 2 + (N % 2)]

        if not window_fun is None:
            win = sgn.get_window(window_fun, N, False)
            w = fft.fft(signal * win, norm="forward")[:N // 2 + (N % 2)]
        else:
            w = fft.fft(signal, norm="forward")[:N // 2 + (N % 2)]
        
        return freqs, w
    
    @classmethod
    def fft(cls, time: np.ndarray, 
                 signal: np.ndarray,
                 window_fun: str = None,
                 time_range: np.ndarray = None, 
                 idx_range: np.ndarray = None) -> tuple[np.ndarray, np.ndarray]:
        """
            Performs one-sided (non-negative frequencies) FFT
        """
        time, signal = signal_manip.slice(time=time, signal=signal, time_range=time_range, idx_range=idx_range)
            
        sample_rate = 1 / np.average(time[1:len(time)] - time[0:len(time)-1])
        
        freqs, w = cls._fft_signal(signal=signal, sample_rate=sample_rate, window_fun=window_fun)

        return freqs, w
    
    @classmethod
    def stft(cls, time: np.ndarray, 
                  signal: np.ndarray,
                  window_fun: str,
                  window_length: int,
                  window_stride: int,
                  backwards_only: bool = True,
                  time_range: np.ndarray = None, 
                  idx_range: np.ndarray = None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        
        time, signal = signal_manip.slice(time=time, signal=signal, time_range=time_range, idx_range=idx_range)
            
        sample_rate = 1 / np.average(time[1:len(time)] - time[0:len(time)-1])

        stft_obj = sgn.ShortTimeFFT(win=sgn.get_window(window_fun, window_length, False), 
                                    hop=window_stride, fs=sample_rate, fft_mode='onesided', scale_to='magnitude')
        
        w = stft_obj.stft(signal)

        times = stft_obj.t(len(signal))[(stft_obj.lower_border_end[1]-1+(window_length % 2)):(stft_obj.p_max(len(signal))-1+(window_length % 2))] + time[0]
        freqs = stft_obj.f
        if backwards_only:
            w = w[:, :(stft_obj.p_max(len(signal)) + stft_obj.p_min - 1 + (window_length % 2))]
        else:
            w = w[:, (stft_obj.lower_border_end[1]-1):(stft_obj.p_max(len(signal))-1)]

        return times, freqs, w

    @staticmethod
    def hilbert(signal: np.ndarray) -> np.ndarray:
        return sgn.hilbert(signal)
    
    @classmethod
    def hilbert_timeseries(cls, time: np.ndarray,
                                signal: np.ndarray,
                                time_range: np.ndarray = None,
                                idx_range: np.ndarray = None) -> tuple[np.ndarray, np.ndarray]:
        
        time, signal = signal_manip.slice(time=time, signal=signal, time_range=time_range, idx_range=idx_range)
        
        analytic_signal = sgn.hilbert(signal)

        return time, analytic_signal
    
    @classmethod
    def acf(cls, time: np.ndarray,
                 signal: np.ndarray,
                 time_range: np.ndarray = None,
                 idx_range: np.ndarray = None) -> tuple[np.ndarray, np.ndarray]:
    
        time, signal = signal_manip.slice(time=time, signal=signal, time_range=time_range, idx_range=idx_range)
        
        acf = stattools.acf(signal, nlags=len(time))
        
        return time, acf
    
    @classmethod
    def psd(cls, time: np.ndarray, 
                 signal: np.ndarray,
                 acf_window_fun: str = None,
                 time_range: np.ndarray = None, 
                 idx_range: np.ndarray = None) -> tuple[np.ndarray, np.ndarray]:
        
        time, acf = cls.acf(time=time, signal=signal, 
                            time_range=time_range, idx_range=idx_range)
        
        sample_rate = 1 / np.average(time[1:len(time)] - time[0:len(time)-1])

        freqs, w = cls._fft_signal(signal=acf, sample_rate=sample_rate, window_fun=acf_window_fun)

        return freqs, w

    @classmethod
    def stats(cls, time: np.ndarray, 
                   list_of_signals: list[np.ndarray], 
                   time_range: np.ndarray = None,
                   idx_range: np.ndarray = None):
        list_of_times = []
        list_of_sgns = []
        list_of_ffts_freqs = []
        list_of_ffts = []
        list_of_acfs_times = []
        list_of_acfs = []
        list_of_psds_freqs = []
        list_of_psds = []

        for signal in list_of_signals:
            sgn_time, sgn = signal_manip.slice(time, signal, time_range=time_range, idx_range=idx_range)
            list_of_times.append(sgn_time)
            list_of_sgns.append(sgn)

            fft_freqs, fft = signal_transform.fft(time, signal, time_range=time_range, idx_range=idx_range)
            list_of_ffts_freqs.append(fft_freqs)
            list_of_ffts.append(fft)

            acf_times, acf = signal_transform.acf(time, signal, time_range=time_range, idx_range=idx_range)
            list_of_acfs_times.append(acf_times)
            list_of_acfs.append(acf)

            psd_freqs, psd = signal_transform.psd(time, signal, time_range=time_range, idx_range=idx_range)
            list_of_psds_freqs.append(psd_freqs)
            list_of_psds.append(psd)

        return (list_of_times, list_of_ffts_freqs, list_of_acfs_times, list_of_psds_freqs), \
               (list_of_sgns,  list_of_ffts,       list_of_acfs,       list_of_psds)
    

class signal_filter():
    @staticmethod
    def butterworth(signal: np.ndarray,
                    sample_rate: float,
                    order: int,
                    critical_freq: float | list[float],
                    btype: str = 'lowpass') -> np.ndarray:
        b, a = sgn.butter(N=order, Wn=critical_freq, btype=btype, fs=sample_rate, output='ba')
        filtered_signal = sgn.lfilter(b, a, signal)
        return filtered_signal
    
    @staticmethod
    def moving_average(signal: np.ndarray,
                       window_length: int,
                       fill_initial_value: float = 0.0):
        ret = np.concatenate((np.full(window_length-1, fill_initial_value), signal))
        ret = np.cumsum(ret, dtype=float)
        ret[window_length:] = ret[window_length:] - ret[:-window_length]
        return ret[window_length - 1:] / window_length
    
    @staticmethod
    def moving_std(signal: np.ndarray, 
                   window_length: int,
                   fill_initial_value: float = 0.0):
        """
            Based on naive calculation of variance found here:
            https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Na%C3%AFve_algorithm
        """
        augmented_signal = np.concatenate((np.full(window_length-1, fill_initial_value), signal))
        cumsum_signal = np.cumsum(augmented_signal)[window_length-1:]
        cumsum_signal[window_length:] = cumsum_signal[window_length:] - cumsum_signal[:-window_length]
        cumsum_signal_sqr = np.cumsum(augmented_signal ** 2)[window_length-1:]
        cumsum_signal_sqr[window_length:] = cumsum_signal_sqr[window_length:] - cumsum_signal_sqr[:-window_length]
        return np.sqrt((cumsum_signal_sqr - cumsum_signal**2 / window_length) / window_length)
    
    @staticmethod
    def moving_rms(signal: np.ndarray,
                   window_length: int):
        ret = np.concatenate((np.zeros(window_length-1), signal))
        ret = ret ** 2
        ret = np.cumsum(ret, dtype=float)
        ret[window_length:] = ret[window_length:] - ret[:-window_length]
        return np.sqrt(ret[window_length - 1:] / window_length)
    
    @staticmethod
    def wiener(signal: np.ndarray,
               window_length: int):
        filtered_signal = sgn.wiener(signal, window_length)
        return filtered_signal
    
    @staticmethod
    def savgol(signal: np.ndarray,
               window_length: int,
               polyorder: int,
               deriv: int = 0,
               delta: float = 1.0,
               mode: str = 'interp',
               cval: float = 0.0):
        filtered_signal = sgn.savgol_filter(x=signal, window_length=window_length, polyorder=polyorder,
                                            deriv=deriv, delta=delta, mode=mode, cval=cval)
        return filtered_signal
