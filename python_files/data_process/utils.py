import os
import operator
import functools
import compress_pickle
import numpy as np


def get_by_path(root, key_list):
    return functools.reduce(operator.getitem, key_list, root)


def set_by_path(root, key_list, value):
    try:
        get_by_path(root, key_list[:-1])[key_list[-1]] = value
    except TypeError as e:
        if hasattr(get_by_path(root, key_list[:-1]), '__dict__'):
            get_by_path(root, key_list[:-1]).__dict__[key_list[-1]] = value
        else:
            raise e


def find_objects_vars(obj, path_list):
    out_list = []

    def search_object_dict(obj, path_list: list):
        if hasattr(obj, '__dict__'):
            for key in obj.__dict__:
                search_object_dict(get_by_path(obj, [key]), [*path_list, key])
        else:
            return out_list.append([*path_list])
    search_object_dict(obj, path_list)

    return out_list


def ax_legend_picking(ax):
    legend_obj = ax.get_legend()
    map_legend_to_ax = {}

    pickradius = 5

    for legend_line, ax_line in zip(legend_obj.get_lines(), ax.get_lines()):
        legend_line.set_picker(pickradius)
        map_legend_to_ax[legend_line] = ax_line

    def on_pick(event):
        # On the pick event, find the original line corresponding to the legend
        # proxy line, and toggle its visibility.
        legend_line = event.artist

        # Do nothing if the source of the event is not a legend line.
        if legend_line not in map_legend_to_ax:
            return

        ax_line = map_legend_to_ax[legend_line]
        visible = not ax_line.get_visible()
        ax_line.set_visible(visible)
        # Change the alpha on the line in the legend, so we can see what lines
        # have been toggled.
        legend_line.set_alpha(1.0 if visible else 0.2)
        ax.figure.canvas.draw()

    ax.figure.canvas.mpl_connect('pick_event', on_pick)


def ax_reset_scale(event, ax):
    if event.key == 'r':
        ax.autoscale()


class ClassifierMetrics:
    @staticmethod
    def confusion_matrix(y_true: np.ndarray | list[int], y_pred: np.ndarray | list[int], ignore_ranges: list[tuple[int, int]] = []):
        """
            Returns a 2D array that corresponds to the confusion
            matrix extracted using 'y_true' and 'y_pred'. First
            dimension corresponds to ground truth labels, second
            shows predicted values.
            
            Each vector should be an array of ints. Assumes 'y_true'
            follows zero-based indexing and the number of different 
            classes is 'max(y_true) + 1'.

            Specific index intervals can be ignored by using
            'ignore_ranges' argument. This argument accepts a list
            of two-element tuples. Each tuple is a region of indices
            (start-stop) that the user wishes to ignore.
        """
        if len(y_true) != len(y_pred):
            raise RuntimeError(f"ClassifierMetrics.confusion_matrix(): 'y_true' and 'y_pred' need to have same length ({len(y_true)} vs {len(y_pred)})")
        
        # Get number of classes
        num_class = max(y_true) + 1

        # Slice vectors if 'ignore_ranges' is not empty
        if ignore_ranges:
            ignore_idx = np.hstack(tuple(np.arange(ran[0], ran[1]+1) for ran in ignore_ranges))
            y_true = np.delete(y_true, ignore_idx)
            y_pred = np.delete(y_pred, ignore_idx)

        # Initialize confusion matrix to zeros
        cm = np.zeros((num_class, num_class), dtype=int)
        # Increment 
        np.add.at(cm, (y_true, y_pred), 1)

        return cm

    @staticmethod
    def overall_metrics(y_true: np.ndarray | list[int], y_pred: np.ndarray | list[int], ignore_ranges: list[tuple[int, int]] = []):
        """
            Calculates classification overall metrics 
            (accuracy, precision, recall, f1_measure,
            macro-averaging for precision, recall).
        """
        # Use confusion matrix as basis for calculating metrics
        cm = ClassifierMetrics.confusion_matrix(y_true=y_true, y_pred=y_pred, ignore_ranges=ignore_ranges)

        return ClassifierMetrics.overall_metrics_from_cm(cm)

    @staticmethod
    def class_metrics(y_true: np.ndarray | list[int], y_pred: np.ndarray | list[int], ignore_ranges: list[tuple[int, int]] = []):
        """
            Calculates classification per-class metrics 
            (accuracy, precision, recall, f1_measure).
        """
        # Use confusion matrix as basis for calculating metrics
        cm = ClassifierMetrics.confusion_matrix(y_true=y_true, y_pred=y_pred, ignore_ranges=ignore_ranges)

        return ClassifierMetrics.class_metrics_from_cm(cm)
        
    @staticmethod
    def overall_metrics_from_cm(cm: np.ndarray):
        """
            Calculates classification overall metrics 
            (accuracy, precision, recall, f1_measure,
            macro-averaging for precision, recall)
            using confusion matrix argument.
        """
        # True Positives for each class corresponds to matrix diagonal
        tp = np.diagonal(cm)
        # False Positive for each class is the sum across each column, minus the True Positive values
        fp = np.sum(cm, axis=0) - tp
        # False Negative for each class is the sum across each row, minus the True Positive values
        fn = np.sum(cm, axis=1) - tp
        # Get total number of samples
        total_samples = np.sum(cm, axis=None)
        
        # Accuracy  = sum(tp) / total_samples
        accuracy   = np.sum(tp) / total_samples
        # Precision = TP / (TP + FP) (macro-averaging)
        precision  = np.mean(np.divide(tp, tp + fp))
        # Recall    = TP / (TP + FN) (macro-averaging)
        recall     = np.mean(np.divide(tp, tp + fn))
        # F1-Score  = 2 * Precision * Recall / (Precision + Recall)
        f1_measure = 2 * precision * recall / (precision + recall)

        return {'accuracy' : accuracy, 'precision' : precision, 'recall' : recall, 'f1_measure' : f1_measure}
    
    @staticmethod
    def class_metrics_from_cm(cm: np.ndarray):
        """
            Calculates classification per-class metrics 
            (accuracy, precision, recall, f1_measure)
            using confusion matrix argument.
        """
        # True Positives for each class corresponds to matrix diagonal
        tp = np.diagonal(cm)
        # False Positive for each class is the sum across each column, minus the True Positive values
        fp = np.sum(cm, axis=0) - tp
        # False Negative for each class is the sum across each row, minus the True Positive values
        fn = np.sum(cm, axis=1) - tp
        # Get total number of samples
        total_samples = np.sum(cm, axis=None)

        # Accuracy  = sum(tp) / total_samples
        accuracies = np.zeros(cm.shape[0])
        # Calculate the accuracy for each class
        for class_idx in range(cm.shape[0]):
            # True negatives are all the samples that are not our current GT class (not the current row) 
            # and were not predicted as the current class (not the current column)
            true_negatives = np.sum(np.delete(np.delete(cm, class_idx, axis=0), class_idx, axis=1))
            
            # True positives are all the samples of our current GT class that were predicted as such
            true_positives = cm[class_idx, class_idx]
            
            # The accuracy for the current class is the ratio between correct predictions to all predictions
            accuracies[class_idx] = (true_positives + true_negatives) / total_samples

        # Precision = TP / (TP + FP) (macro-averaging)
        precisions  = np.divide(tp, tp + fp)
        # Recall    = TP / (TP + FN) (macro-averaging)
        recalls     = np.divide(tp, tp + fn)
        # F1-Score  = 2 * Precision * Recall / (Precision + Recall)
        f1_measures = 2 * precisions * recalls / (precisions + recalls)

        out_dict = {'accuracy' : {}, 'precision' : {}, 'recall' : {}, 'f1_measure' : {}}
        for i in range(cm.shape[0]):
            out_dict['accuracy'][i]   = accuracies[i]
            out_dict['precision'][i]  = precisions[i]
            out_dict['recall'][i]     = recalls[i]
            out_dict['f1_measure'][i] = f1_measures[i]

        return out_dict




class CompressPickleWrapper:
    @staticmethod
    def load(file):
        with open(file, 'rb') as f:
            return compress_pickle.load(f)
        
    @staticmethod
    def save(data, filepath, compression:str=None):
        if os.path.splitext(filepath)[1] != '.pkl':
            raise ValueError("Filepath passed to save_to_pickle() must have extension '.pkl'")
        compress_extension = f'.{compress_pickle.get_default_compression_mapping()[compression]}' if compression is not None else ''
        with open(filepath + f'{compress_extension}', 'wb') as f:
            compress_pickle.dump(data, f, compression=(compression if compression is not None else 'pickle'))




if __name__ == '__main__':
    y_true = [1, 0, 0, 1, 0, 1, 0]
    y_pred = [1, 0, 0, 1, 0, 1, 0]

    print(ClassifierMetrics.confusion_matrix(y_true, y_pred))