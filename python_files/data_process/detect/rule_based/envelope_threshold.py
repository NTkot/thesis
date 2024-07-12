import numpy as np

class EnvelopeThreshold():
    """
        Binary classifier that receives as input 
        the envelope of a signal and classifies 
        each datapoint.
    """
    def __init__(self, start_value_threshold: float, start_counter_threshold: int,
                       stop_value_threshold: float, stop_counter_threshold: int):
        self.start_value_threshold = start_value_threshold
        self.start_counter_threshold = start_counter_threshold
        self.stop_value_threshold = stop_value_threshold
        self.stop_counter_threshold = stop_counter_threshold
        self._init_detect_vars()

    def _init_detect_vars(self):
        self.data_counter = 0
        self.start_idx = -1
        self.stop_idx = -1

        self.start_counter = 0
        self.stop_counter = 0

        self.inside_anomaly = False

        self.anomalies_start_idx = []
        self.anomalies_stop_idx  = []

    def insert(self, value: float):
        # If NOT inside anomaly, we need to check for anomaly start point
        if not self.inside_anomaly:
            if value > self.start_value_threshold:
                if self.start_counter == 0:
                    self.start_idx = self.data_counter

                self.start_counter += 1
                if self.start_counter >= self.start_counter_threshold:
                    self._start_anomaly()
            else:
                self.start_counter = max(0, self.start_counter - 1)
        
        # else if inside anomaly, we need to check for anomaly stop point
        else:
            if value < self.stop_value_threshold:
                if self.stop_counter == 0:
                    self.stop_idx = self.data_counter - 1

                self.stop_counter += 1
                if self.stop_counter >= self.stop_counter_threshold:
                    self._stop_anomaly()
            else:
                self.stop_counter = max(0, self.stop_counter - 1)

        self.data_counter += 1

    def predict(self, values: np.ndarray, ignore_ranges: list[list[int]] = []):
        # Clear previous predict calls
        self._init_detect_vars()
        
        # Insert data serially on the detector
        [self.insert(val) for val in values]

        # Check if specific detected zones should be discarded
        if ignore_ranges:
            # Accomodate negative ignore indices (negative indices -> start from end)
            for i in range(len(ignore_ranges)):
                ignore_ranges[i][0] = ignore_ranges[i][0] if ignore_ranges[i][0] >= 0 else len(values) + ignore_ranges[i][0]
                ignore_ranges[i][1] = ignore_ranges[i][1] if ignore_ranges[i][1] >= 0 else len(values) + ignore_ranges[i][1]

            indices_to_discard = []
            for index, (start, stop) in enumerate(zip(self.anomalies_start_idx, self.anomalies_stop_idx)):
                for rng in ignore_ranges:
                    if (start >= rng[0] and start <= rng[1]) or \
                       (stop  >= rng[0] and stop  <= rng[1]):
                        indices_to_discard.append(index)

            for idx in reversed(indices_to_discard):
                self.anomalies_start_idx.pop(idx)
                self.anomalies_stop_idx.pop(idx)

    def to_time_samples(self, time: np.ndarray) -> list[int]:
        if (not self.anomalies_start_idx) or (not self.anomalies_stop_idx):
            raise RuntimeError("EnvelopeThreshold: You need to run detector first before running 'to_time_samples()'")
        if self.data_counter != len(time):
            raise RuntimeError("EnvelopeThreshold: Length of time vector passed to 'to_time_samples()' is not the same as total data points predicted")
        ret = [0 for _ in range(self.data_counter)]
        for start, stop in zip(self.anomalies_start_idx, self.anomalies_stop_idx):
            for idx in range(start, stop+1):
                ret[idx] = 1
        return ret

    def _start_anomaly(self):
        self.anomalies_start_idx.append(self.start_idx)
        self.start_counter = 0
        self.inside_anomaly = True

    def _stop_anomaly(self):
        self.anomalies_stop_idx.append(self.stop_idx)
        self.stop_counter = 0
        self.inside_anomaly = False

if __name__ == '__main__':
    value_threshold = 0.5
    counter_threshold = 3
    predictor = EnvelopeThreshold(start_value_threshold=value_threshold, start_counter_threshold=counter_threshold,
                                        stop_value_threshold=value_threshold,  stop_counter_threshold=counter_threshold)
    values = np.array([0.19, 0.24, 0.324, 0.38, 0.45, 
                       0.51, 0.52, 0.49, 0.56, 0.76, 
                       0.54, 0.49, 0.51, 0.43, 0.39,
                       0.41, 0.35, 0.31, 0.26, 0.22,
                       0.19, 0.24, 0.324, 0.38, 0.45, 
                       0.51, 0.52, 0.49, 0.56, 0.76, 
                       0.54, 0.49, 0.51, 0.43, 0.39,
                       0.41, 0.35, 0.31, 0.26, 0.22])

    predictor.predict(values)
    print(predictor.anomalies_start_idx)
    print(predictor.anomalies_stop_idx)

    import matplotlib.pyplot as plt
    test_pred_fig = plt.figure(1)
    test_pred_fig.clear()

    plt.plot(values)
    plt.axhline(y=value_threshold, linestyle='--', color='r')
    plt.xticks([i for i in range(len(values))])
    for start, end in zip(predictor.anomalies_start_idx, predictor.anomalies_stop_idx):
        plt.axvspan(xmin=start, xmax=end, color=(0.8, 0.2, 0.2, 0.4))
    plt.title('Test EnvelopeThreshold')

    test_pred_fig.set_size_inches(13,4)
    test_pred_fig.tight_layout(pad=2)

    plt.show()