import os
import numpy as np
from scipy.optimize import curve_fit
from utils import read_from_storage, rmse_metric, plot_sto_file, annotate_plot
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

# ==============================================================================
# test curves to fit

Tds = 0.12  # determined empirically

##
# sigmoid functions


def sigmoid_with_bump(t, B, M, m1, m2, c):
    ''' Sigmoid with "bump" component
    '''
    A = 0
    K = 1
    return A + K / (1.0 + np.exp(B * (t - m1 * Tds))) + \
        M * np.exp(-np.power((t - m2*Tds), 2) / (2 * np.power(c, 2)))


def logistic(t, A, K, C, B, Q, m, v):
    ''' Generalized Logistic function
    '''
    return A + K / np.power((C + Q * np.exp(-B * (t - m * Tds))), v)

##
# comparison curves from bibliography


def anteriorSTA(t):  # todo add reference
    ''' Anterior force component based on the Smooth Transition Assumption (STA)
        by Ren et al.
    '''
    k1 = np.exp(4.0 / 9.0)
    k2 = k1 * np.exp(-16.0 / 9.0) / 2.0
    return k1 * np.exp(-np.power(2.0 * (t - Tds / 3.0) / Tds, 2)) - \
        2.0 * k2 * t / Tds


def componentSTA(t):  # todo add reference
    ''' The force/moment component (other that the anterior force) based on the
        Smooth Transition Assumption (STA) by Ren et al.
    '''
    return np.exp(-np.power((2.0 * t / Tds), 3))
# ===============================================================================


##
# data
subject_dir = os.path.abspath('../TR_3/')
results_dir = os.path.join(subject_dir, 'results_rt/grfm_prediction/')
experiment_grf_file = os.path.join(subject_dir, 'TR_3_GRF.mot')

if not (os.path.isfile(experiment_grf_file)):
    raise RuntimeError('required files do not exist')

##
# read data
experiment_grf = read_from_storage(experiment_grf_file)

r_prefix = 'r_ground_'
l_prefix = 'l_ground_'


# ==============================================================================
# help functions


def detectEvent(v, event):
    ''' Detect HS and TO events in time-series based on transition from zero to
        non-zero values, and vice-versa.
    '''
    if event == 'HS':
        return np.where([v[i-1] != v[i] and v[i-1] == 0
                         for i, x in enumerate(np.array(v) != 0)])[0].tolist()
    elif event == 'TO':
        return np.where([v[i-1] != v[i] and v[i] == 0
                         for i, x in enumerate(np.array(v) != 0)])[0].tolist()
    else:
        raise RuntimeError('Wrong event value')


def prepareMeasuredData(measurements, time, hs_id_list, to_id_list):
    ''' Register the measurements during the Double Support (DS) phases in a
        time-series based on the given lists of indices of heel-strike (HS)
        and toe-off (TO) events.
    '''
    ydata = []
    xdata = []
    for i, (hs_row, to_row) in enumerate(zip(hs_id_list, to_id_list)):
        for hs, to in zip(hs_row, to_row):
            t0 = time[hs]
            y_row = measurements[i][hs:to]
            xdata.extend([t - t0 for t in time[hs:to]])
            ydata.extend([(y) / y_row[0] for y in y_row])
    return ydata, xdata


# ===============================================================================
# prepare data

# find HS and TO events
gait_events = {}
gait_events['rhs'] = detectEvent(experiment_grf[r_prefix + 'force_vx'].values, 'HS')
gait_events['lhs'] = detectEvent(experiment_grf[l_prefix + 'force_vx'].values, 'HS')
gait_events['rto'] = detectEvent(experiment_grf[r_prefix + 'force_vx'].values, 'TO')
gait_events['lto'] = detectEvent(experiment_grf[l_prefix + 'force_vx'].values, 'TO')

# assert equal number of paired (hs,to) indexes
assert len(gait_events['rhs']) == len(gait_events['lto'])
assert len(gait_events['lhs']) == len(gait_events['rto'])

# concatenate data (#! transition happens on the lagging leg)
hs_idx = np.array([gait_events['rhs'], gait_events['lhs']])
to_idx = np.array([gait_events['lto'], gait_events['rto']])

grf_fx = np.array([experiment_grf[l_prefix + 'force_vx'].values,
                   experiment_grf[r_prefix + 'force_vx'].values])
grf_fy = np.array([experiment_grf[l_prefix + 'force_vy'].values,
                   experiment_grf[r_prefix + 'force_vy'].values])
grf_fz = np.array([experiment_grf[l_prefix + 'force_vz'].values,
                   experiment_grf[r_prefix + 'force_vz'].values])
# grf_mx = np.array([experiment_grf[l_prefix + 'torque_x'].values,
#                    experiment_grf[r_prefix + 'torque_x'].values])
# grf_my = np.array([experiment_grf[l_prefix + 'torque_y'].values,
#                    experiment_grf[r_prefix + 'torque_y'].values])
# grf_mz = np.array([experiment_grf[l_prefix + 'torque_z'].values,
#                    experiment_grf[r_prefix + 'torque_z'].values])

grf_data = [grf_fx, grf_fy, grf_fz]
curves = [sigmoid_with_bump, logistic, logistic]
bibliography_curves = [anteriorSTA, componentSTA, componentSTA]
titles = ['Anterior GRF (f_x)', 'Vertical GRF (f_y)', 'Lateral GRF (f_z)']

# ===============================================================================
# fit curve and save results

with PdfPages(results_dir + 'transition_curve_fit.pdf') as pdf,\
        open(results_dir + "curve_parameters.txt", "w") as text_file:
    for i, (data, function) in enumerate(zip(grf_data, curves)):

        # create data vector with measured data
        ydata, xdata = prepareMeasuredData(
            data, experiment_grf['time'].values, hs_idx, to_idx)

        # fit curve to measured data
        params, cov = curve_fit(
            function, xdata, ydata, method='trf')  # lm (default), trf, dogbox

        # write parameters to file
        text_file.write(
            '// {} {} parameters\n'.format(titles[i], function.__name__))
        for j, v in enumerate(params):
            text_file.write('double {} = {};\n'.format(
                function.__code__.co_varnames[1:][j], v))
        text_file.write('\n\n')

        # plot results
        time = np.linspace(0, Tds)
        fig, ax = plt.subplots()
        ax.scatter(xdata, ydata, s=0.1, label='Measured Data')
        ax.plot(time, function(time, *params), color='r',
                linewidth=2, label='Fit Curve')
        ax.plot(time, bibliography_curves[i](time), color='k', linestyle='--',
                linewidth=2, label='Ren et al.')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Normalized Units')
        ax.set_title(titles[i])
        ax.legend()
        ax.grid(True)
        plt.show()
        pdf.savefig(fig)
    plt.close()
