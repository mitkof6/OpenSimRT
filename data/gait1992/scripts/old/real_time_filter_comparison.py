# -*- coding: utf-8 -*-
# Evaluates the accuracy of the real-time filtering method.
#
# author: Dimitar Stanev <jimstanev@gmail.com>
##
import os
import numpy as np
from utils import read_from_storage, plot_sto_file, rmse_metric, annotate_plot
import matplotlib
# matplotlib.rcParams.update({'font.size': 11})
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


# %%
# data

subject_dir = os.path.abspath('../')
output_dir = os.path.join(subject_dir, 'real_time/filtering/')

q_reference_file = os.path.join(subject_dir,
                                'residual_reduction_algorithm/task_Kinematics_q.sto')
q_dot_reference_file = os.path.join(subject_dir,
                                    'residual_reduction_algorithm/task_Kinematics_u.sto')
q_ddot_reference_file = os.path.join(subject_dir,
                                     'residual_reduction_algorithm/task_Kinematics_dudt.sto')

q_filtered_file = os.path.join(output_dir, 'q_filtered.sto')
q_dot_filtered_file = os.path.join(output_dir, 'qDot_filtered.sto')
q_ddot_filtered_file = os.path.join(output_dir, 'qDDot_filtered.sto')

# %%
# read data

q_reference = read_from_storage(q_reference_file, True)
q_dot_reference = read_from_storage(q_dot_reference_file, True)
q_ddot_reference = read_from_storage(q_ddot_reference_file, True)

q_filtered = read_from_storage(q_filtered_file)
q_dot_filtered = read_from_storage(q_dot_filtered_file)
q_ddot_filtered = read_from_storage(q_ddot_filtered_file)

# plot_sto_file(q_reference_file, q_reference_file + '.pdf', 2)
# plot_sto_file(q_filtered_file, q_filtered_file + '.pdf', 2)

# %%
# compare

d_q_total = []
d_u_total = []
d_a_total = []
with PdfPages(output_dir + 'filter_comparison.pdf') as pdf:
    for i in range(1, q_reference.shape[1]):
        # if 'mtp' in q_reference.columns[i] or \
        #    'subtalar' in  q_reference.columns[i] or \
        #    'ankle_angle_l' in  q_reference.columns[i]:
        #     # ankle added because it was too noisy
        #     continue

        d_q = rmse_metric(q_reference.iloc[:, i], q_filtered.iloc[:, i])
        d_u = rmse_metric(q_dot_reference.iloc[:, i], q_dot_filtered.iloc[:, i])
        d_a = rmse_metric(q_ddot_reference.iloc[:, i], q_ddot_filtered.iloc[:, i])
        if not np.isnan(d_q):     # NaN when siganl is zero
            d_q_total.append(d_q)
            d_u_total.append(d_u)
            d_a_total.append(d_a)

        fig, ax = plt.subplots(nrows=1, ncols=3, figsize=(12, 4))

        ax[0].plot(q_reference.time, q_reference.iloc[:, i], label='OpenSim')
        ax[0].plot(q_filtered.time, q_filtered.iloc[:, i], label='filtered')
        ax[0].set_xlabel('time (s)')
        ax[0].set_ylabel('coordinate (deg | m)')
        ax[0].set_title(q_reference.columns[i])
        annotate_plot(ax[0], 'RMSE = ' + str(d_q))
        ax[0].legend(loc='lower left')

        ax[1].plot(q_dot_reference.time, q_dot_reference.iloc[:, i], label='OpenSim')
        ax[1].plot(q_dot_filtered.time, q_dot_filtered.iloc[:, i], label='filtered')
        ax[1].set_xlabel('time (s)')
        ax[1].set_ylabel('speed (deg / s | m / s)')
        ax[1].set_title(q_dot_reference.columns[i])
        annotate_plot(ax[1], 'RMSE = ' + str(d_u))
        # ax[1].legend()

        ax[2].plot(q_ddot_reference.time, q_ddot_reference.iloc[:, i], label='OpenSim')
        ax[2].plot(q_ddot_filtered.time, q_ddot_filtered.iloc[:, i], label='filtered')
        ax[2].set_xlabel('time (s)')
        ax[2].set_ylabel('acceleration (deg / s^2 | m / s^2)')
        ax[2].set_title(q_ddot_reference.columns[i])
        annotate_plot(ax[2], 'RMSE = ' + str(d_a))
        # ax[2].legend()

        fig.tight_layout()
        pdf.savefig(fig)
        plt.close()

print('d_q: μ = ', np.round(np.mean(d_q_total), 3),
      ' σ = ', np.round(np.std(d_q_total, ddof=1), 3))
print('d_u: μ = ', np.round(np.mean(d_u_total), 3),
      ' σ = ', np.round(np.std(d_u_total, ddof=1), 3))
print('d_a: μ = ', np.round(np.mean(d_a_total), 3),
      ' σ = ', np.round(np.std(d_a_total, ddof=1), 3))

with open(output_dir + 'metrics.txt', 'w') as file_handle:
    file_handle.write('RMSE\n')
    file_handle.write('\td_q: μ = ' + str(np.round(np.mean(d_q_total), 3)) +
                      ' σ = ' + str(np.round(np.std(d_q_total, ddof=1), 3)))
    file_handle.write('\n')
    file_handle.write('\td_u: μ = ' + str(np.round(np.mean(d_u_total), 3)) +
                      ' σ = ' + str(np.round(np.std(d_u_total, ddof=1), 3)))
    file_handle.write('\n')
    file_handle.write('\td_a: μ = ' + str(np.round(np.mean(d_a_total), 3)) +
                      ' σ = ' + str(np.round(np.std(d_a_total, ddof=1), 3)))

# %%
