import os
import numpy as np
import pandas as pd
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
gait_events_file = os.path.join(
    subject_dir, 'results_offline/TR_3_heelstrikes.csv')

if not (os.path.isfile(experiment_grf_file) and
        os.path.isfile(right_wrench_rt_file) and
        os.path.isfile(gait_events_file) and
        os.path.isfile(left_wrench_rt_file)):
    raise RuntimeError('required files do not exist')


##
# read data

experiment_grf = read_from_storage(experiment_grf_file)
right_wrench = read_from_storage(right_wrench_rt_file)
left_wrench = read_from_storage(left_wrench_rt_file)
gait_events = pd.read_csv(gait_events_file)

# convert zeros values to NaN for the CoP
experiment_grf.r_ground_force_px = experiment_grf.r_ground_force_px.replace(0.0, float('nan'))
experiment_grf.r_ground_force_py = experiment_grf.r_ground_force_py.replace(0.0, float('nan'))
experiment_grf.r_ground_force_pz = experiment_grf.r_ground_force_pz.replace(0.0, float('nan'))
experiment_grf.l_ground_force_px = experiment_grf.l_ground_force_px.replace(0.0, float('nan'))
experiment_grf.l_ground_force_py = experiment_grf.l_ground_force_py.replace(0.0, float('nan'))
experiment_grf.l_ground_force_pz = experiment_grf.l_ground_force_pz.replace(0.0, float('nan'))
right_wrench.p_x = right_wrench.p_x.replace(0.0, float('nan'))
right_wrench.p_y = right_wrench.p_y.replace(0.0, float('nan'))
right_wrench.p_z = right_wrench.p_z.replace(0.0, float('nan'))
left_wrench.p_x = left_wrench.p_x.replace(0.0, float('nan'))
left_wrench.p_y = left_wrench.p_y.replace(0.0, float('nan'))
left_wrench.p_z = left_wrench.p_z.replace(0.0, float('nan'))


def plotXYZ(gt_data_frame, est_data_frame, id_gt, id_est, hs_events, to_events, title, y_label):
    ''' Helper function for plotting forces/moments/cop
    '''
    fig, ax = plt.subplots(nrows=3, ncols=1, figsize=(8, 8))
    for i in range(3):
        ax[i].plot(gt_data_frame.time, gt_data_frame.iloc[:, i + id_gt],
                   label='Measured')
        ax[i].plot(est_data_frame.time, est_data_frame.iloc[:, i + id_est],
                   '--', label='Predicted')
        ax[i].set_xlabel('time (s)')
        ax[i].set_ylabel(y_label)
        ax[i].set_title(est_data_frame.columns[id_est + i] + ' ' + title)
        ax[i].legend(loc='lower left')
        # ax[i].grid(True)
        for hs in hs_events:
            ax[i].axvline(x=hs, label='HS',color='tab:red',
                          linestyle='--', linewidth=0.5)
        for to in to_events:
            ax[i].axvline(x=to, label='TO', color='tab:cyan',
                          linestyle='--', linewidth=0.5)
    fig.tight_layout()
    pdf.savefig(fig)
    plt.close()


with PdfPages(results_dir + 'grfm_estimation.pdf') as pdf:

    # forces
    # right
    id_gt = experiment_grf.columns.get_loc("r_ground_force_vx")
    id_est = right_wrench.columns.get_loc('f_x')
    plotXYZ(experiment_grf, right_wrench, id_gt, id_est,
            experiment_grf.time.values[gait_events.rhs.values],
            experiment_grf.time.values[gait_events.rto.values],
            'right', 'force (N)')

    # left
    id_gt = experiment_grf.columns.get_loc("l_ground_force_vx")
    id_est = left_wrench.columns.get_loc('f_x')
    plotXYZ(experiment_grf, left_wrench, id_gt, id_est,
            experiment_grf.time.values[gait_events.lhs.values],
            experiment_grf.time.values[gait_events.lto.values],
            'left', 'force (N)')

    # # torques
    # right
    id_gt = experiment_grf.columns.get_loc("r_ground_torque_x")
    id_est = right_wrench.columns.get_loc('tau_x')
    plotXYZ(experiment_grf, right_wrench, id_gt, id_est,
            experiment_grf.time.values[gait_events.rhs.values],
            experiment_grf.time.values[gait_events.rto.values],
            'right', 'moment (Nm)')

    # left
    id_gt = experiment_grf.columns.get_loc("l_ground_torque_x")
    id_est = left_wrench.columns.get_loc('tau_x')
    plotXYZ(experiment_grf, left_wrench, id_gt, id_est,
            experiment_grf.time.values[gait_events.lhs.values],
            experiment_grf.time.values[gait_events.lto.values],
            'left', 'moment (Nm)')

    # # point
    # right
    id_gt = experiment_grf.columns.get_loc("r_ground_force_px")
    id_est = right_wrench.columns.get_loc('p_x')
    plotXYZ(experiment_grf, right_wrench, id_gt, id_est,
            experiment_grf.time.values[gait_events.rhs.values],
            experiment_grf.time.values[gait_events.rto.values],
            'right', 'coordinate (m)')

    # left
    id_gt = experiment_grf.columns.get_loc("l_ground_force_px")
    id_est = left_wrench.columns.get_loc('p_x')
    plotXYZ(experiment_grf, left_wrench, id_gt, id_est,
            experiment_grf.time.values[gait_events.lhs.values],
            experiment_grf.time.values[gait_events.lto.values],
            'left', 'coordinate (m)')

##
