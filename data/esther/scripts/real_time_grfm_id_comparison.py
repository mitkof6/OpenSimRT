import os
import numpy as np
import pandas as pd
from utils import read_from_storage, rmse_metric, plot_sto_file, annotate_plot
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


##
# data

subject_dir = os.path.abspath('../TR_3/')
osim_results = os.path.join(subject_dir, 'results_offline/')
results_dir = os.path.join(subject_dir, 'results_rt/grfm_prediction/')

tau_reference_file = os.path.join(osim_results, 'TR_3_ID_Esther.sto')
tau_rt_file = os.path.join(results_dir, 'tau.sto')

if not (os.path.isfile(tau_reference_file) and
        os.path.isfile(tau_rt_file)):
    raise RuntimeError('required files do not exist')

##
# read data

tau_reference = read_from_storage(tau_reference_file)
tau_rt = read_from_storage(tau_rt_file)

##
# compare

d_tau_total = []
with PdfPages(results_dir + 'id_comparison.pdf') as pdf:
    for i in range(1, tau_reference.shape[1]):

        # find index
        key = tau_reference.columns[i].replace('_moment', '').replace('_force', '')
        j = tau_rt.columns.get_loc(key)

        d_tau = rmse_metric(tau_reference.iloc[:, i], tau_rt.iloc[:, j])
        if not np.isnan(d_tau):     # append only scalar values
            d_tau_total.append(d_tau)

        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(6, 4))

        ax.plot(tau_reference.time, tau_reference.iloc[:, i], label='OpenSim ID')
        ax.plot(tau_rt.time, tau_rt.iloc[:, j], label='Real-time ID')
        ax.set_xlabel('time')
        ax.set_ylabel('Generalized forces (Nm | N)')
        ax.set_title(tau_rt.columns[j])
        annotate_plot(ax, 'RMSE = ' + str(d_tau))
        ax.legend(loc='lower left')

        fig.tight_layout()
        pdf.savefig(fig)
        plt.close()

print('d_tau: μ = ', np.round(np.mean(d_tau_total), 3),
      ' σ = ', np.round(np.std(d_tau_total, ddof=1), 3))

with open(results_dir + 'metrics.txt', 'w') as file_handle:
    file_handle.write('RMSE\n')
    file_handle.write('\td_q: μ = ' + str(np.round(np.mean(d_tau_total), 3)) +
                      ' σ = ' + str(np.round(np.std(d_tau_total, ddof=1), 3)))

##

