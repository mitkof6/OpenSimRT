# Test the performance of the real-time low pass smooth filter.
#
# author: Dimitar Stanev dimitar.stanev@epfl.ch
##
import os
import numpy as np
from utils import read_from_storage, plot_sto_file
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

##
# data

activity = 'run'
subject_dir = os.path.abspath('../data/rajagopal2015')

q_reference_file = os.path.join(subject_dir, 'CMC', activity,
                                'results/cmc_Kinematics_q.sto')
q_dot_reference_file = os.path.join(subject_dir, 'CMC', activity,
                                    'results/cmc_Kinematics_u.sto')
q_ddot_reference_file = os.path.join(subject_dir, 'CMC', activity,
                                     'results/cmc_Kinematics_dudt.sto')

q_filtered_file = os.path.join(subject_dir,
                               'real_time/q_filtered.sto')
q_dot_filtered_file = os.path.join(subject_dir,
                                   'real_time/qDot_filtered.sto')
q_ddot_filtered_file = os.path.join(subject_dir,
                                    'real_time/qDDot_filtered.sto')

output_file = os.path.join(subject_dir,
                           'real_time/filter_comparison_' + activity + '.pdf')

##
# read data

q_reference = read_from_storage(q_reference_file)
q_dot_reference = read_from_storage(q_dot_reference_file)
q_ddot_reference = read_from_storage(q_ddot_reference_file)

q_filtered = read_from_storage(q_filtered_file)
q_dot_filtered = read_from_storage(q_dot_filtered_file)
q_ddot_filtered = read_from_storage(q_ddot_filtered_file)

# plot_sto_file(q_reference_file, q_reference_file + '.pdf', 2)
# plot_sto_file(q_filtered_file, q_filtered_file + '.pdf', 2)

##
# compare

with PdfPages(output_file) as pdf:
    for i in range(1, q_reference.shape[1]):
        fig, ax = plt.subplots(nrows=1, ncols=3,
                               figsize=(8, 3))

        ax[0].plot(q_reference.time, q_reference.iloc[:, i], label='OpenSim')
        ax[0].plot(q_filtered.time, q_filtered.iloc[:, i], label='filtered')
        ax[0].set_xlabel('time')
        ax[0].set_ylabel('generalized coordinates')
        ax[0].set_title(q_reference.columns[i])
        ax[0].legend()

        ax[1].plot(q_dot_reference.time, q_dot_reference.iloc[:, i], label='OpenSim')
        ax[1].plot(q_dot_filtered.time, q_dot_filtered.iloc[:, i], label='filtered')
        ax[1].set_xlabel('time')
        ax[1].set_ylabel('generalized speeds')
        ax[1].set_title(q_dot_reference.columns[i])
        ax[0].legend()

        ax[2].plot(q_ddot_reference.time, q_ddot_reference.iloc[:, i], label='OpenSim')
        ax[2].plot(q_ddot_filtered.time, q_ddot_filtered.iloc[:, i], label='filtered')
        ax[2].set_xlabel('time')
        ax[2].set_ylabel('generalized accelerations')
        ax[2].set_title(q_ddot_reference.columns[i])
        ax[0].legend()

        fig.tight_layout()
        pdf.savefig(fig)
        plt.close()

##
