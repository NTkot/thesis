from .data import LabeledData

class PerSampleFeatures:
    @staticmethod
    def moving_average(data: LabeledData, 
                       window_length: int,
                       fill_initial_value: float = 0.0) -> dict:
        pass

    @staticmethod
    def moving_std(data: LabeledData, 
                   window_length: int,
                   fill_initial_value: float = 0.0) -> dict:
        pass

    @staticmethod
    def moving_rms(data: LabeledData,
                   window_length: int,
                   fill_initial_value: float = 0.0) -> dict:
        pass

    @staticmethod
    def moving_fft(data: LabeledData,
                   window_length: int,
                   top_frequencies_count: int) -> dict:
        pass
