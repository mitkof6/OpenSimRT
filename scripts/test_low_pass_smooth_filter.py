# Test the performance of the real-time low pass smooth filter.
#
# author: Dimitar Stanev dimitar.stanev@epfl.ch
##
import os
import numpy as np
from utils import read_from_storage, plot_sto_file
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


def similarity(s1, s2):
    """
    """
    t1_0 = s1.index[0]
    t1_f = s1.index[-1]
    t2_0 = s2.index[0]
    t2_f = s2.index[-1]
    t_0 = np.round(np.max([t1_0, t2_0]), 3)
    t_f = np.round(np.min([t1_f, t2_f]), 3)
    x = s1[(s1.index >= t_0) & (s1.index <= t_f)].to_list()
    y = s2[(s2.index >= t_0) & (s2.index <= t_f)].to_list()
    return np.round(np.corrcoef(x, y)[0, 1], 3)


def distance(s1, s2):
    """
    """
    # Signals are sampled with the same sampling frequency. Here time
    # series are first aligned.
    t1_0 = s1.index[0]
    t1_f = s1.index[-1]
    t2_0 = s2.index[0]
    t2_f = s2.index[-1]
    t_0 = np.round(np.max([t1_0, t2_0]), 3)
    t_f = np.round(np.min([t1_f, t2_f]), 3)
    x = s1[(s1.index >= t_0) & (s1.index <= t_f)].to_numpy()
    y = s2[(s2.index >= t_0) & (s2.index <= t_f)].to_numpy()
    return np.round(np.sqrt(np.mean((x - y) ** 2)), 3)


##
# data

subject_dir = os.path.abspath('../data/gait2392_new')

q_reference_file = os.path.join(subject_dir,
                                'computed_muscle_controls/task_Kinematics_q.sto')
q_dot_reference_file = os.path.join(subject_dir,
                                    'computed_muscle_controls/task_Kinematics_u.sto')
q_ddot_reference_file = os.path.join(subject_dir,
                                     'computed_muscle_controls/task_Kinematics_dudt.sto')

q_filtered_file = os.path.join(subject_dir,
                               'real_time/filtering/q_filtered.sto')
q_dot_filtered_file = os.path.join(subject_dir,
                                   'real_time/filtering/qDot_filtered.sto')
q_ddot_filtered_file = os.path.join(subject_dir,
                                    'real_time/filtering/qDDot_filtered.sto')

output_dir = os.path.join(subject_dir, 'real_time/filtering/')


##
# read data

q_reference = read_from_storage(q_reference_file, True)
q_dot_reference = read_from_storage(q_dot_reference_file, True)
q_ddot_reference = read_from_storage(q_ddot_reference_file, True)

q_filtered = read_from_storage(q_filtered_file)
q_dot_filtered = read_from_storage(q_dot_filtered_file)
q_ddot_filtered = read_from_storage(q_ddot_filtered_file)

# plot_sto_file(q_reference_file, q_reference_file + '.pdf', 2)
# plot_sto_file(q_filtered_file, q_filtered_file + '.pdf', 2)

##
# compare

d_q_total = []
d_u_total = []
d_a_total = []
with PdfPages(output_dir + 'filter_comparison.pdf') as pdf:
    for i in range(1, q_reference.shape[1]):
        if 'mtp' in q_reference.columns[i] or \
           'subtalar' in  q_reference.columns[i] or \
           'ankle_angle_l' in  q_reference.columns[i]:
            continue

        d_q = distance(q_reference.iloc[:, i], q_filtered.iloc[:, i])
        d_u = distance(q_dot_reference.iloc[:, i], q_dot_filtered.iloc[:, i])
        d_a = distance(q_ddot_reference.iloc[:, i], q_ddot_filtered.iloc[:, i])
        if not np.isnan(d_q):     # NaN when siganl is zero
            d_q_total.append(d_q)
            d_u_total.append(d_u)
            d_a_total.append(d_a)

        fig, ax = plt.subplots(nrows=1, ncols=3,
                               figsize=(8, 3))

        ax[0].plot(q_reference.time, q_reference.iloc[:, i], label='OpenSim')
        ax[0].plot(q_filtered.time, q_filtered.iloc[:, i], label='filtered')
        ax[0].set_xlabel('time')
        ax[0].set_ylabel('generalized coordinates')
        ax[0].set_title(q_reference.columns[i])
        ax[0].text(.05, .95, ' $d = $ ' + str(d_q),
                   transform=ax[0].transAxes, ha="left", va="top")
        ax[0].legend(loc='lower left')

        ax[1].plot(q_dot_reference.time, q_dot_reference.iloc[:, i], label='OpenSim')
        ax[1].plot(q_dot_filtered.time, q_dot_filtered.iloc[:, i], label='filtered')
        ax[1].set_xlabel('time')
        ax[1].set_ylabel('generalized speeds')
        ax[1].set_title(q_dot_reference.columns[i])
        ax[1].text(1.6, .95, ' $d = $ ' + str(d_u),
                   transform=ax[0].transAxes, ha="left", va="top")
        # ax[1].legend()

        ax[2].plot(q_ddot_reference.time, q_ddot_reference.iloc[:, i], label='OpenSim')
        ax[2].plot(q_ddot_filtered.time, q_ddot_filtered.iloc[:, i], label='filtered')
        ax[2].set_xlabel('time')
        ax[2].set_ylabel('generalized accelerations')
        ax[2].set_title(q_ddot_reference.columns[i])
        ax[2].text(3.15, .95, ' $d = $ ' + str(d_a),
                   transform=ax[0].transAxes, ha="left", va="top")
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

##
