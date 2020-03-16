import os
import numpy as np
from utils import read_from_storage, rmse_metric, plot_sto_file, annotate_plot
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

##
# data

subject_dir = os.path.abspath('../')
experiment_dir = os.path.join(subject_dir, 'experimental_data/')
results_dir = os.path.join(subject_dir, 'real_time/grfm_prediction/')

experiment_grf_file = os.path.join(experiment_dir, 'task_grf.mot')
right_wrench_rt_file = os.path.join(results_dir, 'wrench_right.sto')
left_wrench_rt_file = os.path.join(results_dir, 'wrench_left.sto')

if not (os.path.isfile(experiment_grf_file) and
        os.path.isfile(right_wrench_rt_file) and
        os.path.isfile(left_wrench_rt_file)):
    raise RuntimeError('required files do not exist')


##
# read data

experiment_grf= read_from_storage(experiment_grf_file)
right_wrench = read_from_storage(right_wrench_rt_file)
left_wrench = read_from_storage(left_wrench_rt_file)


with PdfPages(results_dir + 'grfm_estimation.pdf') as pdf:

    # forces
    fig, ax = plt.subplots(nrows=3, ncols=1, figsize=(8, 8))

    gt_id_r = experiment_grf.columns.get_loc("ground_force_vx")
    gt_id_l = experiment_grf.columns.get_loc("1_ground_force_vx")
    wr_id = right_wrench.columns.get_loc('f_x')
    wl_id = left_wrench.columns.get_loc('f_x')

    for i in range(3):
        ax[i].plot(experiment_grf.time, experiment_grf.iloc[:, i + gt_id_r],
                    label='Measured')
        ax[i].plot(right_wrench.time, right_wrench.iloc[:, i + wr_id],
            '--', label='Predicted')
        ax[i].set_xlabel('time (s)')
        ax[i].set_ylabel('ground reaction force (N)')
        ax[i].set_title(experiment_grf.columns[gt_id_r + i])
        ax[i].legend(loc='lower left')
        ax[i].grid(True)

    fig.tight_layout()
    pdf.savefig(fig)
    plt.close()

    fig, ax = plt.subplots(nrows=3, ncols=1, figsize=(8, 8))
    for i in range(3):
        ax[i].plot(experiment_grf.time, experiment_grf.iloc[:, i + gt_id_l],
                    label='Measured')
        ax[i].plot(left_wrench.time, left_wrench.iloc[:, i + wl_id],
            '--', label='Predicted')
        ax[i].set_xlabel('time (s)')
        ax[i].set_ylabel('ground reaction force (N)')
        ax[i].set_title(experiment_grf.columns[gt_id_l + i])
        ax[i].legend(loc='lower left')
        ax[i].grid(True)

    fig.tight_layout()
    pdf.savefig(fig)
    plt.close()

    # # torques
    # fig, ax = plt.subplots(nrows=3, ncols=1, figsize=(8, 8))

    # idx1 = grf_labels.index('ground_torque_x')
    # idx2 = grf_labels.index('1_ground_torque_x')
    # idy = grf_est_labels.index('r_ground_torque_x')

    # for i in range(3):
    #     ax[i].plot(grf_data[:, 0], grf_data[:, i + idx1] + grf_data[:,i + idx2],
    #                 label='Measured')
    #     ax[i].plot(grf_est_data[:, 0], grf_est_data[:, i + idy],
    #         '--', label='Predicted')
    #     ax[i].set_xlabel('time (s)')
    #     ax[i].set_ylabel(grf_est_labels[i + idy])
    #     ax[i].grid(True)
    #     ax[i].legend()

    # fig.tight_layout()
    # pdf.savefig(fig)
    # plt.close()



##
