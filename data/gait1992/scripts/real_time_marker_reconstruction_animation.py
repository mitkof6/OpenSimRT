# -*- coding: utf-8 -*-
# Animation of the missing marker reconstruction in marker-based motion capture.
#
# author: Filip Konstantinos <filip.k@ece.upatras.gr>
# %%
import os
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation
import opensim as osim

# %%

# data

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
    raise RuntimeError('required files do not exist')

original_markers = osim.TimeSeriesTableVec3(original_markers_file)
reconstructed_markers = osim.TimeSeriesTable(
    reconstructed_markers_file).packVec3()

# %%

# marker names
marker_labels = original_markers.getColumnLabels()

# time
time = np.array(original_markers.getIndependentColumn())

# total number of frames
num_frames = original_markers.getNumRows()

# init data for animation
t = [] # time
data = [] # marker data
color = [] # marker color

# %%
# reconstructed markers
for label in missing_marker_labels:
    idx = reconstructed_markers.getColumnIndex(label)

    # marker postion
    marker_coord = np.array(
        [reconstructed_markers.getRow(t)[idx].to_numpy() for t in time])

    # append to lists
    data += list(marker_coord)
    t += list(time)
    color += ['r']

# original and missing markers
for label in marker_labels:
    idx = original_markers.getColumnIndex(label)

    # marker postion
    marker_coord = np.array(
        [original_markers.getRow(t)[idx].to_numpy() / 1000 for t in time])

    # append to lists
    data += list(marker_coord)
    t += list(time)
    color += ['y' if label in missing_marker_labels else 'tab:blue']

# as np.array
data = np.array(data)
t = np.array(t)

# %%

# create data frame with animation data
df = {"time": t, "x": data[:, 0], "y": -data[:, 2], "z": data[:, 1]}

# create marker scatter plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
title = ax.set_title('') # get title handle
index = df['time'] == 0
graph = ax.scatter(df['x'][index],
                   df['y'][index],
                   df['z'][index],
                   c=color,
                   edgecolors='none')

# setup custom legend
patch1 = mpatches.Patch(color='tab:blue', label='original marker')
patch2 = mpatches.Patch(color='y', label='missing marker')
patch3 = mpatches.Patch(color='r', label='reconstructed marker')
ax.legend(handles=[patch1, patch2, patch3])

# set axes limits
ax.set_xlim3d(0, 1.2)
ax.set_ylim3d(-0.5, 0.5)
ax.set_zlim3d(-0.05, 1.2)
# plt.xlabel('x')
# plt.ylabel('z')

# %%


# update_graph function
def update_graph(num):
    global graph
    global title
    global df, time

    index = df['time'] == time[num]
    graph._offsets3d = (df['x'][index], df['y'][index], df['z'][index])
    title.set_text('Missing Marker Reconstruction\n Time={}'.format(time[num]))
    return title, graph


# actual animation
ani = matplotlib.animation.FuncAnimation(fig,
                                         update_graph,
                                         frames=num_frames,
                                         interval=60,
                                         blit=False)

plt.show()
# %%
