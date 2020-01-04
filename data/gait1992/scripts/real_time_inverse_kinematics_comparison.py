# -*- coding: utf-8 -*-
# Evaluates the accuracy of the real-time inverse kinematics method.
#
# author: Dimitar Stanev <jimstanev@gmail.com>
##
import os
import numpy as np
from utils import read_from_storage, rmse_metric, plot_sto_file, annotate_plot
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


##
# data

subject_dir = os.path.abspath('../')
output_dir = os.path.join(subject_dir, 'real_time/inverse_kinematics/')

q_reference_file = os.path.join(subject_dir,
                                'inverse_kinematics/task_InverseKinematics.mot')
q_rt_file = os.path.join(output_dir, 'q.sto')

##
# read data

q_reference = read_from_storage(q_reference_file)
q_rt = read_from_storage(q_rt_file)

plot_sto_file(q_rt_file, q_rt_file + '.pdf', 3)

##
# compare

d_q_total = []
with PdfPages(output_dir + 'inverse_kinematics_comparison.pdf') as pdf:
    for i in range(1, q_reference.shape[1]):

        # find index
        j = q_rt.columns.get_loc(q_reference.columns[i])

        # deg to rad for rotational degrees of freedom
        if '_tx' in q_reference.columns[i] or \
           '_ty' in q_reference.columns[i] or \
           '_tz' in q_reference.columns[i]:
            scale = 1
        else:
            scale = 57.295779513

        d_q = rmse_metric(q_reference.iloc[:, i],  scale * q_rt.iloc[:, j])
        if not np.isnan(d_q):     # NaN when siganl is zero
            d_q_total.append(d_q)

        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(6, 4))

        ax.plot(q_reference.time, q_reference.iloc[:, i], label='OpenSim IK')
        ax.plot(q_rt.time, scale * q_rt.iloc[:, j], label='Real-time IK')
        ax.set_xlabel('time')
        ax.set_ylabel('generalized coordinates (deg | m)')
        ax.set_title(q_rt.columns[j])
        annotate_plot(ax, 'RMSE = ' + str(d_q))
        ax.legend(loc='lower left')

        fig.tight_layout()
        pdf.savefig(fig)
        plt.close()

print('d_q: μ = ', np.round(np.mean(d_q_total), 3),
      ' σ = ', np.round(np.std(d_q_total, ddof=1), 3))

with open(output_dir + 'metrics.txt', 'w') as file_handle:
    file_handle.write('RMSE\n')
    file_handle.write('\td_q: μ = ' + str(np.round(np.mean(d_q_total), 3)) +
                      ' σ = ' + str(np.round(np.std(d_q_total, ddof=1), 3)))

##
