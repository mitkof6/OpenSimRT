# -*- coding: utf-8 -*-
# Evaluates the accuracy of the real-time muscle optimization method.
#
# author: Dimitar Stanev <jimstanev@gmail.com>
##
import os
import numpy as np
from utils import read_from_storage, rmse_metric, plot_sto_file, annotate_plot
import matplotlib
matplotlib.rcParams.update({'font.size': 14})
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


##
# data

subject_dir = os.path.abspath('../')
output_dir = os.path.join(subject_dir, 'real_time/muscle_optimization/')

fm_reference_file = os.path.join(subject_dir,
                                 'static_optimization/task_StaticOptimization_force.sto')
fm_rt_file = os.path.join(output_dir, 'fm.sto')
am_rt_file = os.path.join(output_dir, 'am.sto')
tauRes_rt_file = os.path.join(output_dir, 'tauRes.sto')

plot_sto_file(am_rt_file, am_rt_file + '.pdf', 3)
plot_sto_file(tauRes_rt_file, tauRes_rt_file + '.pdf', 3)

##
# read data

fm_reference = read_from_storage(fm_reference_file)
fm_rt = read_from_storage(fm_rt_file)

plot_sto_file(fm_rt_file, fm_rt_file + '.pdf', 3)

##
# compare

d_fm_total = []
with PdfPages(output_dir + 'muscle_optimization_comparison.pdf') as pdf:
    for i in range(1, fm_rt.shape[1]):

        # find index
        key = fm_rt.columns[i]
        j = fm_reference.columns.get_loc(key)

        d_tau = rmse_metric(fm_rt.iloc[:, i], fm_reference.iloc[:, j])
        if not np.isnan(d_tau):     # NaN when signal is zero
            d_fm_total.append(d_tau)

        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(6, 4))

        ax.plot(fm_reference.time, fm_reference.iloc[:, j], label='OpenSim SO')
        ax.plot(fm_rt.time, fm_rt.iloc[:, i], label='Real-time SO')
        ax.set_xlabel('time (s)')
        ax.set_ylabel('actuator force (Nm | N)')
        ax.set_title(fm_reference.columns[j])
        annotate_plot(ax, 'RMSE = ' + str(d_tau))
        ax.legend(loc='lower left')

        fig.tight_layout()
        pdf.savefig(fig)
        plt.close()

print('d_tau: μ = ', np.round(np.mean(d_fm_total), 3),
      ' σ = ', np.round(np.std(d_fm_total, ddof=1), 3))

with open(output_dir + 'metrics.txt', 'w') as file_handle:
    file_handle.write('RMSE\n')
    file_handle.write('\td_q: μ = ' + str(np.round(np.mean(d_fm_total), 3)) +
                      ' σ = ' + str(np.round(np.std(d_fm_total, ddof=1), 3)))

##
