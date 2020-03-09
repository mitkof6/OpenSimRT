import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from util import readMotionFile, readCSV

##
# data

subject_dir = os.getcwd() + '/../data/gait1848/'
# results_osim_dir = subject_dir + 'results_v3.3/'
results_dir = subject_dir + 'results_rt/'
grf_file = subject_dir + 'subject01_walk1_grf.mot'
grf_est_file = results_dir + 'grf_est.csv'

if not (os.path.isfile(grf_file) and
        os.path.isfile(grf_est_file)):
    raise RuntimeError('required files do not exist')


(h, grf_labels, grf_data) = readMotionFile(grf_file)
(grf_est_labels, grf_est_data) = readCSV(grf_est_file)
grf_data = np.array(grf_data)
grf_est_data = np.array(grf_est_data)
grf_step = 6
grf_est_step = 0

##
# compare joint reaction loads

# with PdfPages(results_dir + 'fig/compare_grf.pdf') as pdf:
#     for label in grf_labels:
#         if 'time' in label:
#             continue

#         idx = grf_labels.index(label)
#         idy = grf_est_labels.index(label)

#         fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(8, 8))
#         ax.plot(grf_data[:, 0], grf_data[:, idx],
#                 label='Measured')
#         ax.plot(grf_est_data[:, 0], grf_est_data[:, idy],
#             '--', label='Predicted')
#         ax.set_xlabel('time (s)')
#         ax.set_ylabel(grf_labels[idx])
#         ax.grid(True)
#         ax.legend()

#         fig.tight_layout()
#         pdf.savefig(fig)
#         plt.close()

with PdfPages(results_dir + 'fig/compare_grf.pdf') as pdf:

    # forces
    fig, ax = plt.subplots(nrows=3, ncols=1, figsize=(8, 8))

    idx1 = grf_labels.index('ground_force_vx')
    idx2 = grf_labels.index('1_ground_force_vx')
    idy = grf_est_labels.index('r_ground_force_vx')

    for i in range(3):
        ax[i].plot(grf_data[:, 0], grf_data[:, i + idx1] + grf_data[:,i + idx2],
                    label='Measured')
        ax[i].plot(grf_est_data[:, 0], grf_est_data[:, i + idy],
            '--', label='Predicted')
        ax[i].set_xlabel('time (s)')
        ax[i].set_ylabel(grf_est_labels[i + idy])
        ax[i].grid(True)
        ax[i].legend()

    fig.tight_layout()
    pdf.savefig(fig)
    plt.close()

    # torques
    fig, ax = plt.subplots(nrows=3, ncols=1, figsize=(8, 8))

    idx1 = grf_labels.index('ground_torque_x')
    idx2 = grf_labels.index('1_ground_torque_x')
    idy = grf_est_labels.index('r_ground_torque_x')

    for i in range(3):
        ax[i].plot(grf_data[:, 0], grf_data[:, i + idx1] + grf_data[:,i + idx2],
                    label='Measured')
        ax[i].plot(grf_est_data[:, 0], grf_est_data[:, i + idy],
            '--', label='Predicted')
        ax[i].set_xlabel('time (s)')
        ax[i].set_ylabel(grf_est_labels[i + idy])
        ax[i].grid(True)
        ax[i].legend()

    fig.tight_layout()
    pdf.savefig(fig)
    plt.close()



##
