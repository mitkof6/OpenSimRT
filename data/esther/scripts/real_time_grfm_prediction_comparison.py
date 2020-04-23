import os
import numpy as np
from utils import read_from_storage, rmse_metric, plot_sto_file, annotate_plot
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

##
# data

subject_dir = os.path.abspath('../TR_3/')
experiment_dir = os.path.join(subject_dir, '')
results_dir = os.path.join(subject_dir, 'results_rt/grfm_prediction/')

experiment_grf_file = os.path.join(experiment_dir, 'TR_3_GRF.mot')
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


def plotXYZ(gt_data_frame, est_data_frame, id_gt, id_est, title, y_label):
    fig, ax = plt.subplots(nrows=3, ncols=1, figsize=(8, 8))
    for i in range(3):
        ax[i].plot(gt_data_frame.time, gt_data_frame.iloc[:, i + id_gt],
                    label='Measured')
        ax[i].plot(est_data_frame.time, est_data_frame.iloc[:, i + id_est],
            '--', label='Predicted')
        ax[i].set_xlabel('time (s)')
        ax[i].set_ylabel(y_label)
        ax[i].set_title(est_data_frame.columns[id_est + i] + ' ' + title )
        ax[i].legend(loc='lower left')
        ax[i].grid(True)

    fig.tight_layout()
    pdf.savefig(fig)
    plt.close()

with PdfPages(results_dir + 'grfm_estimation.pdf') as pdf:

    # forces
    # right
    id_gt = experiment_grf.columns.get_loc("r_ground_force_vx")
    id_est = right_wrench.columns.get_loc('f_x')
    plotXYZ(experiment_grf, right_wrench, id_gt, id_est, 'right', 'force (N)')

    # left
    id_gt = experiment_grf.columns.get_loc("l_ground_force_vx")
    id_est = left_wrench.columns.get_loc('f_x')
    plotXYZ(experiment_grf, left_wrench, id_gt, id_est, 'left', 'force (N)')


    # # torques
    # right
    id_gt = experiment_grf.columns.get_loc("r_ground_torque_x")
    id_est = right_wrench.columns.get_loc('tau_x')
    plotXYZ(experiment_grf, right_wrench, id_gt, id_est,'right', 'moment (Nm)')

    # left
    id_gt = experiment_grf.columns.get_loc("l_ground_torque_x")
    id_est = left_wrench.columns.get_loc('tau_x')
    plotXYZ(experiment_grf, left_wrench, id_gt, id_est, 'left', 'moment (Nm)')


    # # point
    # right
    id_gt = experiment_grf.columns.get_loc("r_ground_force_px")
    id_est = right_wrench.columns.get_loc('p_x')
    plotXYZ(experiment_grf, right_wrench, id_gt, id_est, 'right', 'coordinate (m)')

    # left
    id_gt = experiment_grf.columns.get_loc("l_ground_force_px")
    id_est = left_wrench.columns.get_loc('p_x')
    plotXYZ(experiment_grf, left_wrench, id_gt, id_est, 'left', 'coordinate (m)')

##
