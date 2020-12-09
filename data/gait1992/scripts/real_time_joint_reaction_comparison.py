# -*- coding: utf-8 -*-
# Evaluates the accuracy of the real-time inverse dynamics method.
#
# author: Dimitar Stanev <jimstanev@gmail.com>
# %%
import os
import numpy as np
from utils import read_from_storage, rmse_metric, plot_sto_file, annotate_plot
import matplotlib
matplotlib.rcParams.update({'font.size': 12})
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


# %%
# data

subject_dir = os.path.abspath('../')
output_dir = os.path.join(subject_dir, 'real_time/joint_reaction_analysis/')

jr_reference_file = os.path.join(subject_dir,
                                 'joint_reaction_analysis/task_JointReaction_ReactionLoads.sto')
jr_rt_file = os.path.join(output_dir, 'jr.sto')

# %%
# read data

jr_reference = read_from_storage(jr_reference_file)
jr_rt = read_from_storage(jr_rt_file)

plot_sto_file(jr_rt_file, jr_rt_file + '.pdf', 3)

# %%
# compare

d_jr_total = []
with PdfPages(output_dir + 'joint_reaction_comparison.pdf') as pdf:
    for i in range(1, jr_rt.shape[1]):

        # find index
        key = jr_rt.columns[i]
        j = jr_reference.columns.get_loc(key)

        d_jr = rmse_metric(jr_reference.iloc[:, j], jr_rt.iloc[:, i])

        if not np.isnan(d_jr):     # NaN when siganl is zero
            d_jr_total.append(d_jr)

        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(6, 4))

        ax.plot(jr_reference.time, jr_reference.iloc[:, j], label='OpenSim')
        ax.plot(jr_rt.time, jr_rt.iloc[:, i], label='Real-time')
        ax.set_xlabel('time (s)')
        ax.set_ylabel('joint reaction (Nm | N)')
        ax.set_title(jr_rt.columns[i])
        annotate_plot(ax, 'RMSE = ' + str(d_jr))
        ax.legend(loc='lower left')

        fig.tight_layout()
        pdf.savefig(fig)
        plt.close()

print('d_jr: μ = ', np.round(np.mean(d_jr_total), 3),
      ' σ = ', np.round(np.std(d_jr_total, ddof=1), 3))

with open(output_dir + 'metrics.txt', 'w') as file_handle:
    file_handle.write('RMSE\n')
    file_handle.write('\td_q: μ = ' + str(np.round(np.mean(d_jr_total), 3)) +
                      ' σ = ' + str(np.round(np.std(d_jr_total, ddof=1), 3)))

# %%
