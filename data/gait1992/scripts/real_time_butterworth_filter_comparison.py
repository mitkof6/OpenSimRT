# -*- coding: utf-8 -*-
#
# Evaluates the accuracy of the Butterworth filter implementation.
#
# author: Filip Konstantinos <filip.k@ece.upatras.gr>
# %%
import os
import numpy as np
from utils import read_from_storage, annotate_plot
import matplotlib
from scipy import signal
from utils import to_gait_cycle
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from sklearn.metrics import mean_squared_error

matplotlib.rcParams.update({'font.size': 12})
matplotlib.rcParams.update({'legend.framealpha': 0.2})

# %%
# data

subject_dir = os.path.abspath('../')
output_dir = os.path.join(subject_dir, 'real_time/filtering/bw_filter/')

q_reference_file = os.path.join(
    subject_dir, 'residual_reduction_algorithm/task_Kinematics_q.sto')

q_filtered_files = [f for f in os.listdir(output_dir) if f.endswith('.sto')]

for q_filtered_file in q_filtered_files:
    q_filtered_file = os.path.join(output_dir, q_filtered_file)

    # %%
    # read data

    gait_cycle = True

    q_reference = read_from_storage(q_reference_file, True)

    q_filtered = read_from_storage(q_filtered_file)

    # %% reference signal filter parameters

    # get file extension with filter parameters
    s = os.path.basename(os.path.splitext(q_filtered_file)[0]).replace(
        'q_filtered_', '')

    fs = 100  # signal freq
    fc = int(s[s.find('F') + 1:s.rfind('O')])  # target cutoff freq
    filter_order = int(s[s.find('O') + 1:s.rfind('T')])  # filter order

    # filter type
    filter_type_ext = s[s.find('T') + 1:s.rfind('')]
    if (filter_type_ext == 'lp'):
        filt_type = 'low'
    elif (filter_type_ext == 'hp'):
        filt_type = 'high'
    else:
        raise RuntimeError('Wrong filter type.')

    # %%

    if gait_cycle:
        t0 = 0.6  # right heel strike
        tf = 1.83  # next right heel strike
        q_reference = to_gait_cycle(q_reference, t0, tf)
        q_filtered = to_gait_cycle(q_filtered, t0, tf)

    # %%
    # compare

    d_q_total = []
    with PdfPages(output_dir + 'filter_comparison_' + s + '.pdf') as pdf:
        for i in range(1, q_reference.shape[1]):

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

            # filter reference signal with scipy
            b, a = signal.butter(filter_order, (2 * fc) / fs,
                                 filt_type,
                                 analog=False)
            q_ref_filt = signal.lfilter(b, a, q_reference.iloc[:, i].values)

            # get filtered signal with custom implementation
            q_filt = scale * q_filtered.iloc[:, j].values

            # compute error
            d_q = round(mean_squared_error(q_ref_filt, q_filt, squared=False), 2)
            d_q_total.append(d_q)

            # plot
            fig, ax = plt.subplots(nrows=1, ncols=1, figsize=(5, 4))
            ax.plot(q_reference.time, q_ref_filt, label='Python (Scipy)')
            ax.plot(q_filtered.time, q_filt, '--', label='C++ (Custom)')

            if gait_cycle:
                ax.set_xlabel('gait cycle (%)')
            else:
                ax.set_xlabel('Time (seconds)')

            ax.set_xlabel(x_label)
            ax.set_ylabel('coordinate' + position_unit)
            ax.set_title(q_reference.columns[i])
            annotate_plot(ax, 'RMSE = ' + str(d_q))
            ax.legend(loc='lower right')
            fig.patch.set_alpha(1)
            fig.tight_layout()
            pdf.savefig(fig)
            plt.close()

    print('d_q: μ = ', np.round(np.mean(d_q_total), 3), ' σ = ',
          np.round(np.std(d_q_total, ddof=1), 3))

    # %%
