# Evaluates the accuracy of the real-time inverse dynamics method.
#
# author: Dimitar Stanev <jimstanev@gmail.com>
# %%
import os
import numpy as np
from utils import read_from_storage, rmse_metric, plot_sto_file, annotate_plot
from utils import to_gait_cycle
import matplotlib
matplotlib.rcParams.update({'font.size': 12})
matplotlib.rcParams.update({'legend.framealpha': 0.2})
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


# %%
# data

gait_cycle = True

subject_dir = os.path.abspath('../')
output_dir = os.path.join(subject_dir, 'real_time/joint_reaction_analysis/')

jr_reference_file = os.path.join(subject_dir,
                                 'joint_reaction_analysis/task_JointReaction_ReactionLoads.sto')
jr_rt_file = os.path.join(output_dir, 'jr.sto')


# %%
# read data

jr_reference = read_from_storage(jr_reference_file)
jr_rt = read_from_storage(jr_rt_file)
# make very small number zero before plotting
jr_reference[jr_reference.abs() < 1e-9] = 0
jr_rt[jr_reference.abs() < 1e-9] = 0

if gait_cycle:
    t0 = 0.6                        # right heel strike
    tf = 1.83                       # next right heel strike
    jr_reference = to_gait_cycle(jr_reference, t0, tf)
    jr_rt = to_gait_cycle(jr_rt, t0, tf)

plot_sto_file(jr_rt_file, jr_rt_file + '.pdf', 3)

# %%
# compare

d_jr_total = []
with PdfPages(output_dir + 'joint_reaction_comparison_fm.pdf') as pdf:
    for i in range(1, jr_rt.shape[1]):

        # find index
        key = jr_rt.columns[i]
        j = jr_reference.columns.get_loc(key)

        d_jr = rmse_metric(jr_reference.iloc[:, j], jr_rt.iloc[:, i])

        if not np.isnan(d_jr):     # NaN when siganl is zero
            d_jr_total.append(d_jr)

        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(5, 4))

        ax.plot(jr_reference.time, jr_reference.iloc[:, j],
                label='OpenSim')
        ax.plot(jr_rt.time, jr_rt.iloc[:, i], label='Real-time',
                linestyle='--')
        ax.set_title(jr_rt.columns[i])
        if gait_cycle:
            ax.set_xlabel('gait cycle (%)')
        else:
            ax.set_xlabel('time (s)')

        if jr_rt.columns[i][-3::] in ['_mx', '_my', '_mz']:
            ax.set_ylabel('joint reaction moment (N m)')
        elif jr_rt.columns[i][-3::] in ['_fx', '_fy', '_fz']:
            ax.set_ylabel('joint reaction force (N)')
        else:
            ax.set_ylabel('joint reaction position (m)')

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
