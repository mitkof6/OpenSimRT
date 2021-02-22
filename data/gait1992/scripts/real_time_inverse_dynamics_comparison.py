# -*- coding: utf-8 -*-
# Evaluates the accuracy of the real-time inverse dynamics method.
#
# author: Dimitar Stanev <jimstanev@gmail.com>
##
import os
import numpy as np
from utils import read_from_storage, rmse_metric, plot_sto_file, annotate_plot
import matplotlib.pyplot as plt
from utils import to_gait_cycle
from matplotlib.backends.backend_pdf import PdfPages
import matplotlib
matplotlib.rcParams.update({'font.size': 12})
matplotlib.rcParams.update({'legend.framealpha': 0.2})


##
# data

gait_cycle = True

subject_dir = os.path.abspath('../')
output_dir = os.path.join(subject_dir, 'real_time/inverse_dynamics/')

tau_reference_file = os.path.join(subject_dir,
                                  'inverse_dynamics/task_InverseDynamics.sto')
tau_rt_file = os.path.join(output_dir, 'tau.sto')

##
# read data

tau_reference = read_from_storage(tau_reference_file)
tau_rt = read_from_storage(tau_rt_file)

if gait_cycle:
    t0 = 0.6                        # right heel strike
    tf = 1.83                       # next right heel strike
    tau_reference = to_gait_cycle(tau_reference, t0, tf)
    tau_rt = to_gait_cycle(tau_rt, t0, tf)

plot_sto_file(tau_rt_file, tau_rt_file + '.pdf', 3)

##
# compare

d_tau_total = []
with PdfPages(output_dir + 'inverse_dynamics_comparison.pdf') as pdf:
    for i in range(1, tau_reference.shape[1]):

        # find index
        key = tau_reference.columns[i].replace('_moment', '').replace('_force', '')
        j = tau_rt.columns.get_loc(key)

        d_tau = rmse_metric(tau_reference.iloc[:, i], tau_rt.iloc[:, j])
        if not np.isnan(d_tau):     # NaN when siganl is zero
            d_tau_total.append(d_tau)

        is_deg = True
        if tau_rt.columns[i] in ['pelvis_tx', 'pelvis_ty', 'pelvis_tz']:
            is_deg = False

        units = ''
        if is_deg:
            units = ' (N m)'
        else:
            units = ' (N)'

        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(5, 4))

        ax.plot(tau_reference.time, tau_reference.iloc[:, i],
                label='OpenSim ID')
        ax.plot(tau_rt.time, tau_rt.iloc[:, j], label='Real-time ID',
                linestyle='--')
        if gait_cycle:
            ax.set_xlabel('gait cycle (%)')
        else:
            ax.set_xlabel('time (s)')

        ax.set_ylabel('generalized forces' + units)
        ax.set_title(tau_rt.columns[j])
        annotate_plot(ax, 'RMSE = ' + str(d_tau))
        ax.legend(loc='lower left')

        fig.tight_layout()
        pdf.savefig(fig)
        plt.close()

print('d_tau: μ = ', np.round(np.mean(d_tau_total), 3),
      ' σ = ', np.round(np.std(d_tau_total, ddof=1), 3))

with open(output_dir + 'metrics.txt', 'w') as file_handle:
    file_handle.write('RMSE\n')
    file_handle.write('\td_q: μ = ' + str(np.round(np.mean(d_tau_total), 3)) +
                      ' σ = ' + str(np.round(np.std(d_tau_total, ddof=1), 3)))

##
