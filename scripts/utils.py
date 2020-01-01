# Utility functions.
#
# author: Dimitar Stanev jimstanev@gmail.com
##
import re
import opensim
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


def osim_array_to_list(array):
    """Convert OpenSim::Array<T> to Python list.
    """
    temp = []
    for i in range(array.getSize()):
        temp.append(array.get(i))

    return temp


def read_from_storage(file_name, to_filter=False):
    """Read OpenSim.Storage files.

    Parameters
    ----------
    file_name: (string) path to file

    Returns
    -------
    tuple: (labels, time, data)
    """
    sto = opensim.Storage(file_name)
    sto.resample(0.01, 3)
    if to_filter:
        sto.lowpassFIR(4, 6)

    labels = osim_array_to_list(sto.getColumnLabels())
    time = opensim.ArrayDouble()
    sto.getTimeColumn(time)
    time = osim_array_to_list(time)
    data = []
    for i in range(sto.getSize()):
        temp = osim_array_to_list(sto.getStateVector(i).getData())
        temp.insert(0, time[i])
        data.append(temp)

    df = pd.DataFrame(data, columns=labels)
    df.index = df.time
    return df


def index_containing_substring(list_str, pattern):
    """For a given list of strings finds the index of the element that
    contains the substring.

    Parameters
    ----------
    list_str: list of str

    pattern: str
         pattern


    Returns
    -------
    indices: list of int
         the indices where the pattern matches

    """
    return [i for i, item in enumerate(list_str)
            if re.search(pattern, item)]


def plot_sto_file(file_name, plot_file, plots_per_row=4, pattern=None,
                  title_function=lambda x: x):
    """Plots the .sto file (OpenSim) by constructing a grid of subplots.

    Parameters
    ----------
    sto_file: str
        path to file
    plot_file: str
        path to store result
    plots_per_row: int
        subplot columns
    pattern: str, optional, default=None
        plot based on pattern (e.g. only pelvis coordinates)
    title_function: lambda
        callable function f(str) -> str
    """
    df = read_from_storage(file_name)
    labels = df.columns.to_list()
    data = df.to_numpy()

    if pattern is not None:
        indices = index_containing_substring(labels, pattern)
    else:
        indices = range(1, len(labels))

    n = len(indices)
    ncols = int(plots_per_row)
    nrows = int(np.ceil(float(n) / plots_per_row))
    pages = int(np.ceil(float(nrows) / ncols))
    if ncols > n:
        ncols = n

    with PdfPages(plot_file) as pdf:
        for page in range(0, pages):
            fig, ax = plt.subplots(nrows=ncols, ncols=ncols,
                                   figsize=(8, 8))
            ax = ax.flatten()
            for pl, col in enumerate(indices[page * ncols ** 2:page *
                                             ncols ** 2 + ncols ** 2]):
                ax[pl].plot(data[:, 0], data[:, col])
                ax[pl].set_title(title_function(labels[col]))

            fig.tight_layout()
            pdf.savefig(fig)
            plt.close()


def adjust_model_mass(model_file, mass_change):
    """Given a required mass change adjust all body masses accordingly.

    """
    rra_model = opensim.Model(model_file)
    rra_model.setName('Rajagopal2015_adjusted')
    state = rra_model.initSystem()
    current_mass = rra_model.getTotalMass(state)
    new_mass = current_mass + mass_change
    mass_scale_factor = new_mass / current_mass
    for body in rra_model.updBodySet():
        body.setMass(mass_scale_factor * body.getMass())

    # save model with adjusted body masses
    rra_model.printToXML(model_file)


def subject_specific_isometric_force(generic_model_file, subject_model_file,
                                     height_generic, height_subject):
    """Adjust the max isometric force of the subject-specific model based on results
    from Handsfield et al. 2014 [1] (equation from Fig. 5A). Function adapted
    from Rajagopal et al. 2015 [2].

    Given the height and mass of the generic and subject models, we can
    calculate the total muscle volume [1]:

    V_total = 47.05 * mass * height + 1289.6

    Since we can calculate the muscle volume and the optimal fiber length of the
    generic and subject model, respectively, we can calculate the force scale
    factor to scale the maximum isometric force of each muscle:

    scale_factor = (V_total_subject / V_total_generic) / (l0_subject / l0_generic)

    F_max_i = scale_factor * F_max_i

    [1] http://dx.doi.org/10.1016/j.jbiomech.2013.12.002
    [2] http://dx.doi.org/10.1109/TBME.2016.2586891

    """
    model_generic = opensim.Model(generic_model_file)
    state_generic = model_generic.initSystem()
    mass_generic = model_generic.getTotalMass(state_generic)
    
    model_subject = opensim.Model(subject_model_file)
    state_subject = model_subject.initSystem()
    mass_subject = model_subject.getTotalMass(state_subject)

    # formula for total muscle volume
    V_total_generic = 47.05 * mass_generic * height_generic + 1289.6
    V_total_subject = 47.05 * mass_subject * height_subject + 1289.6

    for i in range(0, model_subject.getMuscles().getSize()):
        muscle_generic = model_generic.updMuscles().get(i)
        muscle_subject = model_subject.updMuscles().get(i)

        l0_generic = muscle_generic.getOptimalFiberLength()
        l0_subject = muscle_subject.getOptimalFiberLength()

        force_scale_factor = (V_total_subject / V_total_generic) / (l0_subject /
                                                                    l0_generic)
        muscle_subject.setMaxIsometricForce(force_scale_factor *
                                            muscle_subject.getMaxIsometricForce())

    model_subject.printToXML(subject_model_file)


