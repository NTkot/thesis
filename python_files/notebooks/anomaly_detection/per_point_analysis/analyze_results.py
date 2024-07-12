import os
import json
import matplotlib.pyplot as plt

from data_process.file_scripts.common import prompt_user
from data_process.utils import get_by_path

def prompt_result_files() -> str:
    file_folder = os.path.dirname(__file__)

    results_files = []
    for base, folders, files in os.walk(file_folder):
        for file in files:
            if ('results' in file) and ('.json' in file):
                results_files.append(os.path.join(base, file))

    if results_files:
        reduced_path_results_files = [s.replace(file_folder, '') for s in results_files]
        indices = prompt_user(reduced_path_results_files, "Select pickle file to annotate:")
        if len(indices) != 1:
            raise RuntimeError('Select exactly one results file')
        result_file_selected = results_files[indices[0]]
        return result_file_selected
    else:
        raise RuntimeError(f"Could not auto-detect results files on path '{file_folder}'")
    
def subdict_has_same_values(big, small):
    def flatten_dict_list(obj, path):
        if isinstance(obj, (str, int, float)):
            return [(path, obj)]
        elif isinstance(obj, dict):
            ret = []
            for k, v in obj.items():
                ret.extend(flatten_dict_list(v, path + [k]))
            return ret
        elif isinstance(obj, list):            
            ret = []
            for (n, elem) in enumerate(obj):
                ret.extend(flatten_dict_list(elem, path + [str(n)]))
            return ret
        else:
            raise ValueError("supported value types: str, int, float")
    
    flatted_small = flatten_dict_list(small, [])

    paths  = [entry[0] for entry in flatted_small]
    values = [entry[1] for entry in flatted_small]

    flag = True
    for p, v in zip(paths, values):
        big_v = get_by_path(big, p)
        if v != big_v:
            flag = False
            break
    return flag


if __name__ == '__main__':
    # Find results filepaths
    results_file = prompt_result_files()

    # Get results
    with open(results_file, 'r') as file:
        results = json.load(file)

    # Get scalar metric for each experiment
    metric_path = input('Enter dictionary path for metric to use in evaluation (use \'/\' to separate path levels): ')
    metric_path = metric_path.split('/')
    metric_values = [get_by_path(experiment, ['metrics'] + metric_path) for experiment in results]
    sort_order = sorted(range(len(metric_values)), key=lambda i: metric_values[i], reverse=True)
    metric_values = [metric_values[i] for i in sort_order]

    # Get params for each experiment
    hyperparams = [get_by_path(experiment, ['params']) for experiment in results]
    hyperparams = [hyperparams[i] for i in sort_order]

    # Get only hyperparams that match contain the following dictionary
    subset_params = \
    {
        "filter": {
            "gyro": {
                "wiener": 21,
                "hp_order": 2,
                "hp_freq": 2
            }
        }
    }
    mask = [subdict_has_same_values(params, subset_params) for params in hyperparams]
    hyperparams   = [params for params, m in zip(hyperparams, mask)   if m]
    metric_values = [value for value, m   in zip(metric_values, mask) if m]

    # Get number of top results to show
    plot_results_number = 25

    # Plot results
    fig = plt.figure(1)
    bars = plt.bar(list(range(plot_results_number)), metric_values[:plot_results_number])
    plt.ylabel('/'.join(metric_path))
    plt.ylim((0.975*min(metric_values[:plot_results_number]), 1.01*max(metric_values[:plot_results_number])))

    ax : plt.Axes = fig.get_axes()[0]
    annot = ax.annotate("", xy=(0,0), xytext=(-20,20),textcoords="offset points",
                    bbox=dict(boxstyle="round", fc="black", ec="b", lw=2),
                    arrowprops=dict(arrowstyle="->"))
    annot.set_visible(False)

    class HoverWrapper():
        def __init__(self):
            self.previous_hovered_bar = None

        def hover(self, event):
            vis = annot.get_visible()
            if event.inaxes == ax:
                for bar in bars:
                    cont, ind = bar.contains(event)
                    if cont and bar != self.previous_hovered_bar:
                        index = int(bar.get_x() + bar.get_width() / 2.)
                        self.update_annot(bar, index)
                        annot.set_visible(True)
                        fig.canvas.draw_idle()
                        print(json.dumps(hyperparams[index], indent=2))
                        return
            if vis:
                annot.set_visible(False)
                fig.canvas.draw_idle()

        def update_annot(self, bar, index):
            x = bar.get_x()+bar.get_width()/2.
            y = bar.get_y()+bar.get_height()
            annot.xy = (x,y)
            text = f"{metric_values[index]:.4f}"
            annot.set_text(text)
            annot.get_bbox_patch().set_alpha(0.4)

    hover_obj = HoverWrapper()
    fig.canvas.mpl_connect("motion_notify_event", hover_obj.hover)

    fig.set_size_inches(13,6)
    fig.tight_layout(pad=2)
    plt.show()
