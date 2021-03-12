# Evaluates the accuracy of the real-time filtering method.
#
# author: Dimitar Stanev <jimstanev@gmail.com>
# %%
import os
import numpy as np
from utils import read_from_storage, plot_sto_file, rmse_metric, annotate_plot
import matplotlib
from utils import to_gait_cycle
matplotlib.rcParams.update({'font.size': 12})
matplotlib.rcParams.update({'legend.framealpha': 0.2})
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

folder = 'proposed_filter/'
q_filtered_file = os.path.join(output_dir, folder, 'q_filtered.sto')
q_dot_filtered_file = os.path.join(output_dir, folder, 'qDot_filtered.sto')
q_ddot_filtered_file = os.path.join(output_dir, folder, 'qDDot_filtered.sto')

q_filtered_sp_file = os.path.join(output_dir, 'spatial_filter/q_filtered.sto')
q_dot_filtered_sp_file = os.path.join(output_dir, 'spatial_filter/qDot_filtered.sto')
q_ddot_filtered_sp_file = os.path.join(output_dir, 'spatial_filter/qDDot_filtered.sto')

# %%
# read data

gait_cycle = True

q_reference = read_from_storage(q_reference_file, True)
q_dot_reference = read_from_storage(q_dot_reference_file, True)
q_ddot_reference = read_from_storage(q_ddot_reference_file, True)

q_filtered = read_from_storage(q_filtered_file)
q_dot_filtered = read_from_storage(q_dot_filtered_file)
q_ddot_filtered = read_from_storage(q_ddot_filtered_file)

q_filtered_sp = read_from_storage(q_filtered_sp_file)
q_dot_filtered_sp = read_from_storage(q_dot_filtered_sp_file)
q_ddot_filtered_sp = read_from_storage(q_ddot_filtered_sp_file)

if gait_cycle:
    t0 = 0.6                        # right heel strike
    tf = 1.83                       # next right heel strike
    q_reference = to_gait_cycle(q_reference, t0, tf)
    q_dot_reference = to_gait_cycle(q_dot_reference, t0, tf)
    q_ddot_reference = to_gait_cycle(q_ddot_reference, t0, tf)
    q_filtered = to_gait_cycle(q_filtered, t0, tf)
    q_dot_filtered = to_gait_cycle(q_dot_filtered, t0, tf)
    q_ddot_filtered = to_gait_cycle(q_ddot_filtered, t0, tf)
    q_filtered_sp = to_gait_cycle(q_filtered_sp, t0, tf)
    q_dot_filtered_sp = to_gait_cycle(q_dot_filtered_sp, t0, tf)
    q_ddot_filtered_sp = to_gait_cycle(q_ddot_filtered_sp, t0, tf)


plot_sto_file(q_reference_file, q_reference_file + '.pdf', 2)
plot_sto_file(q_filtered_file, q_filtered_file + '.pdf', 2)

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
        key = q_reference.columns[i]
        j = q_filtered.columns.get_loc(key)

        if key[-2::] in ['tx', 'ty', 'tz']:
            scale = 1
        else:
            scale = 57.29578

        x_label = ''
        if gait_cycle:
            x_label = 'gait cycle (%)'
        else:
            x_label = 'time (s)'

        is_deg = True
        if q_reference.columns[i] in ['pelvis_tx', 'pelvis_ty', 'pelvis_tz']:
            is_deg = False

        position_unit = ''
        velocity_unit = ''
        acceleration_unit = ''
        if is_deg:
            position_unit = ' (deg)'
            velocity_unit = ' (deg / s)'
            acceleration_unit = ' (deg / $s^2$)'
        else:
            position_unit = ' (m)'
            velocity_unit = ' (m / s)'
            acceleration_unit = ' (m / $s^2$)'

        d_q = rmse_metric(q_reference.iloc[:, i], scale * q_filtered.iloc[:, j])
        d_u = rmse_metric(q_dot_reference.iloc[:, i], scale * q_dot_filtered.iloc[:, j])
        d_a = rmse_metric(q_ddot_reference.iloc[:, i], scale * q_ddot_filtered.iloc[:, j])
        d_q_sp = rmse_metric(q_reference.iloc[:, i], scale * q_filtered_sp.iloc[:, j])
        d_u_sp = rmse_metric(q_dot_reference.iloc[:, i], scale * q_dot_filtered_sp.iloc[:, j])
        d_a_sp = rmse_metric(q_ddot_reference.iloc[:, i], scale * q_ddot_filtered_sp.iloc[:, j])
        if not np.isnan(d_q):     # NaN when siganl is zero
            d_q_total.append(d_q)
            d_u_total.append(d_u)
            d_a_total.append(d_a)

        fig, ax = plt.subplots(nrows=1, ncols=3, figsize=(13, 4))

        ax[0].plot(q_reference.time, q_reference.iloc[:, i],
                   label='OpenSim')
        ax[0].plot(q_filtered.time, scale * q_filtered.iloc[:, j],
                   label='Proposed filter', linestyle='--')
        ax[0].plot(q_filtered_sp.time, scale * q_filtered_sp.iloc[:, j],
                   label='Spatial filter', linestyle=':')
        ax[0].set_xlabel(x_label)
        ax[0].set_ylabel('coordinate' + position_unit)
        ax[0].set_title(q_reference.columns[i])
        annotate_plot(ax[0], 'RMSE = ' + str(d_q))
        annotate_plot(ax[0], 'RMSE = ' + str(d_q_sp), 'upper right')
        ax[0].legend(loc='lower left')

        ax[1].plot(q_dot_reference.time, q_dot_reference.iloc[:, i],
                   label='OpenSim')
        ax[1].plot(q_dot_filtered.time, scale * q_dot_filtered.iloc[:, j],
                   label='Proposed filter', linestyle='--')
        ax[1].plot(q_dot_filtered_sp.time, scale * q_dot_filtered_sp.iloc[:, j],
                   label='Spatial filter', linestyle=':')
        ax[1].set_xlabel(x_label)
        ax[1].set_ylabel('speed' + velocity_unit)
        ax[1].set_title(q_dot_reference.columns[i])
        annotate_plot(ax[1], 'RMSE = ' + str(d_u))
        annotate_plot(ax[1], 'RMSE = ' + str(d_u_sp), 'upper right')
        # ax[1].legend()

        ax[2].plot(q_ddot_reference.time, q_ddot_reference.iloc[:, i],
                   label='OpenSim')
        ax[2].plot(q_ddot_filtered.time, scale * q_ddot_filtered.iloc[:, j],
                   label='Proposed filter', linestyle='--')
        ax[2].plot(q_ddot_filtered_sp.time, scale * q_ddot_filtered_sp.iloc[:, j],
                   label='Spatial filter', linestyle=':')
        ax[2].set_xlabel(x_label)
        ax[2].set_ylabel('acceleration' + acceleration_unit)
        ax[2].set_title(q_ddot_reference.columns[i])
        annotate_plot(ax[2], 'RMSE = ' + str(d_a))
        annotate_plot(ax[2], 'RMSE = ' + str(d_a_sp), 'upper right')
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
