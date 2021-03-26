# Evaluates the accuracy of the real-time modules in the pipeline method, in
# comparison with the resutls from the individual test files.
#
# author: Filip Konstantinos filip.k@ece.upatras.gr
# %%
import os
import numpy as np
from utils import read_from_storage, rmse_metric, annotate_plot
from utils import to_gait_cycle
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import matplotlib
matplotlib.rcParams.update({'font.size': 12})
matplotlib.rcParams.update({'legend.framealpha': 0.2})

# %%
# data

gait_cycle = True

subject_dir = os.path.abspath('../real_time')
output_dir = os.path.join(subject_dir, 'pipeline/')

q_reference_file = os.path.join(subject_dir, 'inverse_dynamics/q_filtered.sto')
tau_reference_file = os.path.join(subject_dir, 'inverse_dynamics/tau.sto')
fm_reference_file = os.path.join(subject_dir, 'muscle_optimization/fm.sto')
jr_reference_file = os.path.join(subject_dir, 'joint_reaction_analysis/jr.sto')

q_pipeline_file = os.path.join(output_dir, 'q.sto')
tau_pipeline_file = os.path.join(output_dir, 'tau.sto')
fm_pipeline_file = os.path.join(output_dir, 'fm.sto')
jr_pipeline_file = os.path.join(output_dir, 'jr.sto')

# %%
# read data

q_reference = read_from_storage(q_reference_file)
q_pipeline = read_from_storage(q_pipeline_file)

tau_reference = read_from_storage(tau_reference_file)
tau_pipeline = read_from_storage(tau_pipeline_file)

fm_reference = read_from_storage(fm_reference_file)
fm_pipeline = read_from_storage(fm_pipeline_file)

jr_reference = read_from_storage(jr_reference_file)
jr_pipeline = read_from_storage(jr_pipeline_file)

if gait_cycle:
    t0 = 0.6  # right heel strike
    tf = 1.83  # next right heel strike
    q_reference = to_gait_cycle(q_reference, t0, tf)
    q_pipeline = to_gait_cycle(q_pipeline, t0, tf)

    tau_reference = to_gait_cycle(tau_reference, t0, tf)
    tau_pipeline = to_gait_cycle(tau_pipeline, t0, tf)

    fm_reference = to_gait_cycle(fm_reference, t0, tf)
    fm_pipeline = to_gait_cycle(fm_pipeline, t0, tf)

    jr_reference = to_gait_cycle(jr_reference, t0, tf)
    jr_pipeline = to_gait_cycle(jr_pipeline, t0, tf)

# %%
# compare

d_q_total = []
with PdfPages(os.path.join(output_dir,
                           'inverse_kinematics_comparison.pdf')) as pdf:
    for i in range(1, q_reference.shape[1]):

        # find index
        j = q_pipeline.columns.get_loc(q_reference.columns[i])

        is_deg = True
        if q_reference.columns[i] in ['pelvis_tx', 'pelvis_ty', 'pelvis_tz']:
            is_deg = False

        position_unit = ''
        if is_deg:
            position_unit = ' (deg)'
        else:
            position_unit = ' (m)'

        d_q = rmse_metric(q_reference.iloc[:, i], q_pipeline.iloc[:, j])
        if not np.isnan(d_q):  # NaN when siganl is zero
            d_q_total.append(d_q)

        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(5, 4))

        ax.plot(q_reference.time, q_reference.iloc[:, i], label='Test IK')
        ax.plot(q_pipeline.time,
                q_pipeline.iloc[:, j],
                label='Pipeline IK',
                linestyle='--')
        if gait_cycle:
            ax.set_xlabel('gait cycle (%)')
        else:
            ax.set_xlabel('time (s)')

        ax.set_ylabel('generalized coordinates' + position_unit)
        ax.set_title(q_pipeline.columns[j])
        annotate_plot(ax, 'RMSE = ' + str(d_q))
        ax.legend(loc='lower left')

        fig.tight_layout()
        pdf.savefig(fig)
        plt.close()

print('d_q: μ = ', np.round(np.mean(d_q_total), 3), ' σ = ',
      np.round(np.std(d_q_total, ddof=1), 3))

with open(os.path.join(output_dir, 'metrics_q.txt'), 'w') as file_handle:
    file_handle.write('RMSE\n')
    file_handle.write('\td_q: μ = ' + str(np.round(np.mean(d_q_total), 3)) +
                      ' σ = ' + str(np.round(np.std(d_q_total, ddof=1), 3)))

d_tau_total = []
with PdfPages(os.path.join(output_dir,
                           'inverse_dynamics_comparison.pdf')) as pdf:
    for i in range(1, tau_reference.shape[1]):

        # find index
        key = tau_reference.columns[i].replace('_moment',
                                               '').replace('_force', '')
        j = tau_pipeline.columns.get_loc(key)

        d_tau = rmse_metric(tau_reference.iloc[:, i], tau_pipeline.iloc[:, j])
        if not np.isnan(d_tau):  # NaN when siganl is zero
            d_tau_total.append(d_tau)

        is_deg = True
        if tau_pipeline.columns[i] in ['pelvis_tx', 'pelvis_ty', 'pelvis_tz']:
            is_deg = False

        units = ''
        if is_deg:
            units = ' (N m)'
        else:
            units = ' (N)'

        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(5, 4))

        ax.plot(tau_reference.time, tau_reference.iloc[:, i], label='Test ID')
        ax.plot(tau_pipeline.time,
                tau_pipeline.iloc[:, j],
                label='Pipeline ID',
                linestyle='--')
        if gait_cycle:
            ax.set_xlabel('gait cycle (%)')
        else:
            ax.set_xlabel('time (s)')

        ax.set_ylabel('generalized forces' + units)
        ax.set_title(tau_pipeline.columns[j])
        annotate_plot(ax, 'RMSE = ' + str(d_tau))
        ax.legend(loc='lower left')

        fig.tight_layout()
        pdf.savefig(fig)
        plt.close()

print('d_tau: μ = ', np.round(np.mean(d_tau_total), 3), ' σ = ',
      np.round(np.std(d_tau_total, ddof=1), 3))

with open(os.path.join(output_dir, 'metrics_tau.txt'), 'w') as file_handle:
    file_handle.write('RMSE\n')
    file_handle.write('\td_q: μ = ' + str(np.round(np.mean(d_tau_total), 3)) +
                      ' σ = ' + str(np.round(np.std(d_tau_total, ddof=1), 3)))

d_fm_total = []
with PdfPages(os.path.join(output_dir,
                           'muscle_optimization_comparison.pdf')) as pdf:
    for i in range(1, fm_pipeline.shape[1]):

        # find index
        key = fm_pipeline.columns[i]
        j = fm_reference.columns.get_loc(key)

        d_tau = rmse_metric(fm_pipeline.iloc[:, i], fm_reference.iloc[:, j])
        if not np.isnan(d_tau):  # NaN when signal is zero
            d_fm_total.append(d_tau)

        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(5, 4))

        ax.plot(fm_reference.time, fm_reference.iloc[:, j], label='Test SO')
        ax.plot(fm_pipeline.time,
                fm_pipeline.iloc[:, i],
                label='Pipeline SO',
                linestyle='--')
        if gait_cycle:
            ax.set_xlabel('gait cycle (%)')
        else:
            ax.set_xlabel('time (s)')

        ax.set_ylabel('muscle force (N)')
        ax.set_title(fm_reference.columns[j])
        annotate_plot(ax, 'RMSE = ' + str(d_tau))
        ax.legend(loc='upper right')

        fig.tight_layout()
        pdf.savefig(fig)
        plt.close()

print('d_fm: μ = ', np.round(np.mean(d_fm_total), 3), ' σ = ',
      np.round(np.std(d_fm_total, ddof=1), 3))

with open(os.path.join(output_dir, 'metrics_fm.txt'), 'w') as file_handle:
    file_handle.write('RMSE\n')
    file_handle.write('\td_q: μ = ' + str(np.round(np.mean(d_fm_total), 3)) +
                      ' σ = ' + str(np.round(np.std(d_fm_total, ddof=1), 3)))

d_jr_total = []
with PdfPages(os.path.join(output_dir,
                           'joint_reaction_comparison_fm.pdf')) as pdf:
    for i in range(1, jr_pipeline.shape[1]):

        # find index
        key = jr_pipeline.columns[i]
        j = jr_reference.columns.get_loc(key)

        d_jr = rmse_metric(jr_reference.iloc[:, j], jr_pipeline.iloc[:, i])

        if not np.isnan(d_jr):  # NaN when siganl is zero
            d_jr_total.append(d_jr)

        fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(5, 4))

        ax.plot(jr_reference.time, jr_reference.iloc[:, j], label='Test JR')
        ax.plot(jr_pipeline.time,
                jr_pipeline.iloc[:, i],
                label='Pipeline JR',
                linestyle='--')
        ax.set_title(jr_pipeline.columns[i])
        if gait_cycle:
            ax.set_xlabel('gait cycle (%)')
        else:
            ax.set_xlabel('time (s)')

        if jr_pipeline.columns[i][-3::] in ['_mx', '_my', '_mz']:
            ax.set_ylabel('joint reaction moment (N m)')
        elif jr_pipeline.columns[i][-3::] in ['_fx', '_fy', '_fz']:
            ax.set_ylabel('joint reaction force (N)')
        else:
            ax.set_ylabel('joint reaction position (m)')

        annotate_plot(ax, 'RMSE = ' + str(d_jr))
        ax.legend(loc='lower left')

        fig.tight_layout()
        pdf.savefig(fig)
        plt.close()

print('d_jr: μ = ', np.round(np.mean(d_jr_total), 3), ' σ = ',
      np.round(np.std(d_jr_total, ddof=1), 3))

with open(os.path.join(output_dir, 'metrics_jr.txt'), 'w') as file_handle:
    file_handle.write('RMSE\n')
    file_handle.write('\td_q: μ = ' + str(np.round(np.mean(d_jr_total), 3)) +
                      ' σ = ' + str(np.round(np.std(d_jr_total, ddof=1), 3)))

# %%
