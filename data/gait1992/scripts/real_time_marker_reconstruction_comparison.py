# -*- coding: utf-8 -*-
# Evaluates the accuracy of the real-time missing marker reconstruction method.
#
# author: Filip Konstantinos <filip.k@ece.upatras.gr>
# %%
import os
import numpy as np
import matplotlib.pyplot as plt
import opensim as osim
from matplotlib.backends.backend_pdf import PdfPages

# %%

# data

subject_dir = os.path.abspath('../')
results_dir = os.path.join(subject_dir, 'real_time/marker_reconstruction')
original_markers_file = os.path.join(subject_dir, 'experimental_data/task.trc')
reconstructed_markers_file = os.path.join(results_dir,
                                          'reconstructed_markers.sto')

if not (os.path.isfile(original_markers_file)
        and os.path.isfile(reconstructed_markers_file)):
    raise RuntimeError('Required files do not exist.')

# load files
original_markers = osim.TimeSeriesTableVec3(original_markers_file)
reconstructed_markers = osim.TimeSeriesTable(
    reconstructed_markers_file).packVec3()

# %%

# column labels
column_labels = reconstructed_markers.getColumnLabels()

# time
time = np.array(reconstructed_markers.getIndependentColumn())

# %%

with PdfPages(os.path.join(results_dir,
                           'marker_reconstruction_comparison.pdf')) as pdf:
    for column_name in column_labels:
        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(5, 4))

        # get column ids with the same label
        idx = original_markers.getColumnIndex(column_name)
        idy = reconstructed_markers.getColumnIndex(column_name)

        # fetch original marker positions (values in mm)
        original_marker = np.round(
            np.array(
                [original_markers.getRow(t)[idx].to_numpy()
                 for t in time]) / 1000, 4)

        # fetch reconstructed_marker positions (values in m)
        reconstructed_marker = np.round(
            np.array([
                reconstructed_markers.getRow(t)[idy].to_numpy() for t in time
            ]), 4)

        # compute reconstruction error
        err = np.linalg.norm(original_marker - reconstructed_marker, axis=1)

        # plot
        ax.plot(time, err)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Distance Error (m)')
        ax.set_title(column_labels[idy])
        ax.grid(True)
        fig.tight_layout()
        pdf.savefig(fig)
        plt.close()
# %%
