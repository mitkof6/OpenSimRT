# Evaluates the accuracy of the real-time missing marker reconstruction method.
#
# author: Filip Konstantinos <filip.k@ece.upatras.gr>
# %%
import os
import numpy as np
import matplotlib.pyplot as plt
from utils import to_gait_cycle, read_from_storage, rmse_metric, annotate_plot
from matplotlib.backends.backend_pdf import PdfPages
import matplotlib
matplotlib.rcParams.update({'font.size': 10})
matplotlib.rcParams.update({'legend.framealpha': 0.2})

# %%

# data

gait_cycle = True
skip_valid_markers = True

occlusion_start_time = 1.1
occlusion_duration = 0.7

missing_marker_labels = [
    "R.ASIS", "R.Thigh.Rear", "R.Toe.Tip", "R.Shank.Upper", "L.Shank.Upper",
    "L.Thigh.Rear", "L.Toe.Tip"
]

subject_dir = os.path.abspath('../')
results_dir = os.path.join(subject_dir, 'real_time/marker_reconstruction')
original_markers_file = os.path.join(subject_dir, 'experimental_data/task.trc')
reconstructed_markers_file = os.path.join(results_dir,
                                          'reconstructed_markers.sto')

if not (os.path.isfile(original_markers_file)
        and os.path.isfile(reconstructed_markers_file)):
    raise RuntimeError('Required files do not exist.')

original_markers = read_from_storage(original_markers_file)
reconstructed_markers = read_from_storage(reconstructed_markers_file)

# %%

if gait_cycle:
    t0 = 0.6  # right heel strike
    tf = 1.83  # next right heel strike
    original_markers = to_gait_cycle(original_markers, t0, tf)
    reconstructed_markers = to_gait_cycle(reconstructed_markers, t0, tf)

    occlusion_start_perc = 100.0 / (tf - t0) * (occlusion_start_time - t0)
    occlusion_stop_perc = 100.0 / (tf - t0) * (occlusion_start_time +
                                               occlusion_duration - t0)

# %%

d_q_total = []
with PdfPages(os.path.join(results_dir,
                           'marker_reconstruction_comparison.pdf')) as pdf:
    reconstructed_markers.columns = [
        c.replace('_1', '_x') for c in reconstructed_markers.columns
    ]
    reconstructed_markers.columns = [
        c.replace('_2', '_y') for c in reconstructed_markers.columns
    ]
    reconstructed_markers.columns = [
        c.replace('_3', '_z') for c in reconstructed_markers.columns
    ]
    # for each column in column labels:
    for i in range(1, reconstructed_markers.shape[1]):
        # find index in original
        j = original_markers.columns.get_loc(reconstructed_markers.columns[i])

        # skip valid markers
        if skip_valid_markers:
            if (not reconstructed_markers.columns[i][:-2]
                    in missing_marker_labels):
                continue

        scale = 1000
        position_unit = ' (mm)'

        d_q = rmse_metric(scale * reconstructed_markers.iloc[:, i],
                          original_markers.iloc[:, j])
        if not np.isnan(d_q):  # NaN when siganl is zero
            d_q_total.append(d_q)

        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(5, 4))

        ax.plot(original_markers.time,
                original_markers.iloc[:, j],
                label='Original')
        ax.plot(reconstructed_markers.time,
                scale * reconstructed_markers.iloc[:, i],
                label='Reconstructed',
                linestyle='--')
        ax.axvspan(occlusion_start_perc,
                   occlusion_stop_perc,
                   label="Occlusion Period",
                   color="tab:blue",
                   alpha=0.3)
        if gait_cycle:
            ax.set_xlabel('gait cycle (%)')
        else:
            ax.set_xlabel('time (s)')

        ax.set_ylabel('position' + position_unit)
        ax.set_title(original_markers.columns[j])
        annotate_plot(ax, 'RMSE = ' + str(d_q))
        ax.legend(loc='lower left')

        fig.tight_layout()
        pdf.savefig(fig)
        plt.close()

print('d_q: μ = ', np.round(np.mean(d_q_total), 3), ' σ = ',
      np.round(np.std(d_q_total, ddof=1), 3))

with open(results_dir + '_metrics.txt', 'w') as file_handle:
    file_handle.write('RMSE\n')
    file_handle.write('\td_q: μ = ' + str(np.round(np.mean(d_q_total), 3)) +
                      ' σ = ' + str(np.round(np.std(d_q_total, ddof=1), 3)))
# %%
