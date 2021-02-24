# -*- coding: utf-8 -*-
#
# Evaluates the accuracy of the real-time ground reaction forces and moments and
# center of pressure estimation method by comparing the estimation results with
# the measured data.
#
# author: Filip Konstantinos <filip.k@ece.upatras.gr>

# %%
import os
import numpy as np
from utils import read_from_storage, \
    annotate_plot, to_gait_cycle
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from sklearn.metrics import mean_squared_error

params = {
    'legend.fontsize': 10,
    'legend.handlelength': 2,
    'legend.framealpha': 0.2
}
plt.rcParams.update(params)

# data

subject_dir = os.path.abspath('../')
results_dir = os.path.join(subject_dir, 'real_time/grfm_prediction/')
experimental_dir = os.path.join(subject_dir, 'experimental_data/')

right_wrench_rt_file = os.path.join(results_dir, 'wrench_right.sto')
left_wrench_rt_file = os.path.join(results_dir, 'wrench_left.sto')
experimental_grfm_file = os.path.join(experimental_dir, 'task_grf.mot')

r_prefix = 'ground_'
l_prefix = '1_ground_'

if not (os.path.isfile(right_wrench_rt_file)
        and os.path.isfile(left_wrench_rt_file)
        and os.path.isfile(experimental_grfm_file)):
    raise RuntimeError('required files do not exist')

# read data
right_wrench = read_from_storage(right_wrench_rt_file)
left_wrench = read_from_storage(left_wrench_rt_file)
experimental_grfm = read_from_storage(experimental_grfm_file,
                                      True)  # resamples at 0.01

# Dropping last n rows using drop
n = 5
experimental_grfm = experimental_grfm.head(-n)

num_rows = experimental_grfm.shape[0]
# extend measured grfm in file
simulation_loops = 2
df = experimental_grfm
for i in range(simulation_loops - 1):
    experimental_grfm = experimental_grfm.append(df, ignore_index=True)
experimental_grfm.time = list(
    map(lambda x: x / 100.0, range(0, num_rows * simulation_loops, 1)))

lower_lim = 0.6 + 2.45
upper_lim = 1.83 + 2.45

experimental_grfm = experimental_grfm[
    (experimental_grfm["time"].apply(lambda x: x) >= lower_lim)
    & (experimental_grfm["time"].apply(lambda x: x) <= upper_lim)]

right_wrench = right_wrench[
    (right_wrench["time"].apply(lambda x: x) >= lower_lim)
    & (right_wrench["time"].apply(lambda x: x) <= upper_lim)]

left_wrench = left_wrench[
    (left_wrench["time"].apply(lambda x: x) >= lower_lim)
    & (left_wrench["time"].apply(lambda x: x) <= upper_lim)]

# convert zeros values to NaN for the Co
right_wrench.p_x = right_wrench.p_x.replace(0.0, float('nan'))
right_wrench.p_y = right_wrench.p_y.replace(0.0, float('nan'))
right_wrench.p_z = right_wrench.p_z.replace(0.0, float('nan'))
left_wrench.p_x = left_wrench.p_x.replace(0.0, float('nan'))
left_wrench.p_y = left_wrench.p_y.replace(0.0, float('nan'))
left_wrench.p_z = left_wrench.p_z.replace(0.0, float('nan'))

# %%


def detectEvent(v, event):
    ''' Detect HS and TO events in time-series based on transition from zero to
        non-zero values, and vice-versa.
    '''
    if event == 'HS':
        return np.where([
            v[i - 1] != v[i] and v[i - 1] == 0
            for i, x in enumerate(np.array(v) != 0)
        ])[0].tolist()
    elif event == 'TO':
        return np.where([
            v[i - 1] != v[i] and v[i] == 0
            for i, x in enumerate(np.array(v) != 0)
        ])[0].tolist()
    else:
        raise RuntimeError('Wrong event value')


def plotXYZ(gt_data_frame, est_data_frame, id_gt, id_est, hs_events, to_events,
            title, y_label):
    ''' Helper function for plotting forces/moments/cop
    '''
    for i in range(3):

        a = np.array(est_data_frame.iloc[:, i + id_est])
        b = np.array(gt_data_frame.iloc[:, i + id_gt])

        err = round(
            mean_squared_error(b[~np.isnan(a)], a[~np.isnan(a)],
                               squared=False), 2)

        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(5, 4))

        ax.plot(gt_data_frame.time, b, label='Measured')
        ax.plot(est_data_frame.time, a, '--', label='Predicted')
        ax.set_xlabel('gait cycle (%)')
        ax.set_ylabel(y_label)
        ax.set_title(title[i])
        ax.legend(loc='lower right')
        # # ax.grid(True)
        for hs in hs_events:
            ax.axvline(x=hs,
                       label='HS',
                       color='tab:red',
                       linestyle='--',
                       linewidth=0.5)
        for to in to_events:
            ax.axvline(x=to,
                       label='TO',
                       color='tab:cyan',
                       linestyle='--',
                       linewidth=0.5)
        annotate_plot(ax, 'RMSE = ' + str(err))
        fig.patch.set_alpha(1)
        fig.tight_layout()
        pdf.savefig(fig)
        plt.close()


titles = [[
    'Anterior Reaction Force $F_x$ (R)', 'Vertical Reaction Force $F_y$ (R)',
    'Lateral Reaction Force $F_z$ (R)'
],
          [
              'Anterior Reaction Force $F_x$ (L)',
              'Vertical Reaction Force $F_y$ (L)',
              'Lateral Reaction Force $F_z$ (L)'
          ],
          [
              'Anterior Reaction Moment $M_x$ (R)',
              'Vertical Reaction Moment $M_y$ (R)',
              'Lateral Reaction Moment  $M_z$ (R)'
          ],
          [
              'Anterior Reaction Moment $M_x$ (L)',
              'Vertical Reaction Moment $M_y$ (L)',
              'Lateral Reaction Moment  $M_z$ (L)'
          ],
          [
              'Anterior CoP Coordinate $Px$ (R)',
              'Vertical CoP Coordinate $Py$ (R)',
              'Lateral CoP Coordinate  $P_z$ (R)'
          ],
          [
              'Anterior CoP Coordinate $Px$ (L)',
              'Vertical CoP Coordinate $Py$ (L)',
              'Lateral CoP Coordinate  $P_z$ (L)'
          ]]

# find HS and TO events
gait_events = {}
gait_events['rhs'] = detectEvent(
    experimental_grfm[r_prefix + 'force_vx'].values, 'HS')
gait_events['lhs'] = detectEvent(
    experimental_grfm[l_prefix + 'force_vx'].values, 'HS')
gait_events['rto'] = detectEvent(
    experimental_grfm[r_prefix + 'force_vx'].values, 'TO')
gait_events['lto'] = detectEvent(
    experimental_grfm[l_prefix + 'force_vx'].values, 'TO')

# %%

gait_cycle = True
total_time = 2.45
if gait_cycle:
    t0 = 0.6 + total_time  # right heel strike
    tf = 1.83 + total_time  # next right heel strike
    experimental_grfm = to_gait_cycle(experimental_grfm, t0, tf)
    right_wrench = to_gait_cycle(right_wrench, t0, tf)
    left_wrench = to_gait_cycle(left_wrench, t0, tf)

with PdfPages(results_dir + 'grfm_estimation.pdf') as pdf:

    # forces
    id_est = right_wrench.columns.get_loc('f_x')
    id_gt = experimental_grfm.columns.get_loc('ground_force_vx')
    plotXYZ(experimental_grfm, right_wrench, id_gt, id_est, [], [], titles[0],
            'force (N)')

    # left
    id_est = left_wrench.columns.get_loc('f_x')
    id_gt = experimental_grfm.columns.get_loc('1_ground_force_vx')
    plotXYZ(experimental_grfm, left_wrench, id_gt, id_est, [], [], titles[1],
            'force (N)')

    # # torques
    # right
    id_est = right_wrench.columns.get_loc('tau_x')
    id_gt = experimental_grfm.columns.get_loc('ground_torque_x')
    plotXYZ(experimental_grfm, right_wrench, id_gt, id_est, [], [], titles[2],
            'moment (N m)')

    # left
    id_est = left_wrench.columns.get_loc('tau_x')
    id_gt = experimental_grfm.columns.get_loc('1_ground_torque_x')
    plotXYZ(experimental_grfm, left_wrench, id_gt, id_est, [], [], titles[3],
            'moment (N m)')

    # # point
    # right
    id_est = right_wrench.columns.get_loc('p_x')
    id_gt = experimental_grfm.columns.get_loc('ground_force_px')
    plotXYZ(experimental_grfm, right_wrench, id_gt, id_est, [], [], titles[4],
            'coordinate (m)')

    # left
    id_est = left_wrench.columns.get_loc('p_x')
    id_gt = experimental_grfm.columns.get_loc('1_ground_force_px')
    plotXYZ(experimental_grfm, left_wrench, id_gt, id_est, [], [], titles[5],
            'coordinate (m)')

##
