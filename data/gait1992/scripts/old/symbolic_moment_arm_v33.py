# \brief Calculates the symbolic expression of the muscle moment arm of an
#  OpenSim .osim model. The moment arm is sampled and approximated by a
#  multivariate polynomial, so that higher order derivatives can be
#  computed. This implementation works with OpenSim v3.3 API.
#
# Dependencies: opensim, matplotlib, numpy, sympy, multipolyfit, tqdm
#
# @author Dimitar Stanev (stanev@ece.upatras.gr)
import os
import csv
import pickle
import opensim
import numpy as np
import sympy as sp
import operator  # used in sorted
from tqdm import tqdm
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # for projection='3d'
from matplotlib.backends.backend_pdf import PdfPages
from multipolyfit import multipolyfit, mk_sympy_function
plt.rcParams['font.size'] = 13

###############################################################################
# utilities

def cartesian(arrays, out=None):
    """Generate a cartesian product of input arrays.

    Parameters
    ----------

    arrays: list of array-like
        1-D arrays to form the cartesian product of.

    out: ndarray
        Array to place the cartesian product in.

    Returns
    -------

    out: ndarray
        2-D array of shape (M, len(arrays)) containing cartesian products
        formed of input arrays.

    Examples
    --------

    >>> cartesian(([1, 2, 3], [4, 5], [6, 7]))
    array([[1, 4, 6],
           [1, 4, 7],
           [1, 5, 6],
           [1, 5, 7],
           [2, 4, 6],
           [2, 4, 7],
           [2, 5, 6],
           [2, 5, 7],
           [3, 4, 6],
           [3, 4, 7],
           [3, 5, 6],
           [3, 5, 7]])
    """
    arrays = [np.asarray(x) for x in arrays]
    dtype = arrays[0].dtype

    n = np.prod([x.size for x in arrays])
    if out is None:
        out = np.zeros([n, len(arrays)], dtype=dtype)

    m = n / arrays[0].size
    out[:, 0] = np.repeat(arrays[0], m)
    if arrays[1:]:
        cartesian(arrays[1:], out=out[0:m, 1:])
        for j in xrange(1, arrays[0].size):
            out[j * m:(j + 1) * m, 1:] = out[0:m, 1:]

    return out


def construct_coordinate_grid(model, coordinates, N=5):
    """Given n coordinates get the coordinate range and generate a
    coordinate grid of combinations using cartesian product.

    Parameters
    ----------

    model: opensim.Model

    coordinates: list of string

    N: int (default=5)
        the number of points per coordinate

    Returns
    -------

    sampling_grid: np.array
        all combination of coordinates

    """
    sampling_grid = []
    for coordinate in coordinates:
        min_range = model.getCoordinateSet().get(coordinate).getRangeMin()
        max_range = model.getCoordinateSet().get(coordinate).getRangeMax()
        sampling_grid.append(np.linspace(min_range, max_range, N,
                                         endpoint=True))

    return cartesian(sampling_grid)


def find_intermediate_joints(origin_body, insertion_body, model_tree, joints):
    """Finds the intermediate joints between two bodies.

    Parameters
    ----------

    origin_body: string
        first body in the model tree

    insertion_body: string
        last body in the branch

    model_tree: list of dictionary relations {parent, joint, child}

    joints: list of strings
        intermediate joints
    """
    if origin_body == insertion_body:
        return True

    children = filter(lambda x: x['parent'] == origin_body, model_tree)
    for child in children:
        found = find_intermediate_joints(child['child'], insertion_body,
                                         model_tree, joints)
        if found:
            joints.append(child['joint'])
            return True

    return False


def visualize_moment_arm(moment_arm_coordinate, muscle, coordinates,
                         sampling_dict, model_coordinates, model_muscles, R,
                         pdf):
    """Visualize moment arm as 2D or 3D plot.

    Parameters
    ----------

    moment_arm_coordinate: string
        which moment arm (coordinate)

    muscle: string
        which muscle

    coordinates: list of strings
        which coordinates affect the moment arm variable (one or two only)

    sampling_dict: dictionary
        calculated from calculate_moment_arm_symbolically

    model_coordinates: dictionary
        coordinate names and their corresponding indices in the model

    model_muscles: dictionary
        muscle names and their corresponding indices in the model

    R: symbolic moment arm matrix

    pdf: PdfPages
    """
    if isinstance(coordinates, str):
        # coordinates = sampling_dict[muscle]['coordinates']
        sampling_grid = sampling_dict[muscle]['sampling_grid']
        moment_arm = sampling_dict[muscle]['moment_arm']
        idx = coordinates.index(moment_arm_coordinate)
        poly = R[model_muscles[muscle],
                 model_coordinates[moment_arm_coordinate]]
        moment_arm_poly = np.array([
            poly.subs(dict(zip(poly.free_symbols, x))) for x in sampling_grid
        ], np.float)

        fig = plt.figure()
        ax = fig.gca()
        ax.plot(
            sampling_grid[:, idx], moment_arm[:, idx] * 100.0, 'rx',
            label='sampled')
        ax.plot(sampling_grid[:, idx], moment_arm_poly * 100.0, 'b-',
                label='analytical')
        ax.set_xlabel(coordinates + ' (rad)')
        ax.set_ylabel(moment_arm_coordinate + ' (cm)')
        ax.set_title(muscle)
        ax.legend()
        fig.tight_layout()
        pdf.savefig(fig)
        plt.close()
    elif isinstance(coordinates, list) and len(coordinates) == 2:
        # coordinates = sampling_dict[muscle]['coordinates']
        sampling_grid = sampling_dict[muscle]['sampling_grid']
        moment_arm = sampling_dict[muscle]['moment_arm']
        idx = coordinates.index(moment_arm_coordinate)
        poly = R[model_muscles[muscle], model_coordinates[
            moment_arm_coordinate]]
        # poly.free_symbols is not used because it may not preserve order
        poly_symbols = [sp.Symbol(x) for x in coordinates]
        moment_arm_poly = np.array([
            poly.subs(dict(zip(poly_symbols, x))) for x in sampling_grid
        ], np.float)

        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.scatter(
            sampling_grid[:, 0],
            sampling_grid[:, 1],
            moment_arm[:, idx] * 100.0,
            label='sampled',
            color='r')
        surf = ax.plot_trisurf(
            sampling_grid[:, 0],
            sampling_grid[:, 1],
            moment_arm_poly * 100.0,
            label='analytical',
            facecolor='b',
            edgecolor='k',
            linewidth=0.1,
            alpha=0.5,
            antialiased=True)
        surf._facecolors2d = surf._facecolors3d
        surf._edgecolors2d = surf._edgecolors3d
        ax.set_xlabel(coordinates[0] + ' (rad)')
        ax.set_ylabel(coordinates[1] + ' (rad)')
        ax.set_zlabel(moment_arm_coordinate + ' (cm)')
        ax.set_title(muscle)
        ax.legend()
        fig.tight_layout()
        pdf.savefig(fig)
        plt.close()
    else:
        return


def calculate_moment_arm_symbolically(model_file, results_dir):
    """Calculate the muscle moment arm matrix symbolically for a
    particular OpenSim model.

    """
    print('Calculating...')

    # parse csv
    muscle_coordinates = {}
    with open(results_dir + 'muscle_coordinates.csv') as csv_file:
        reader = csv.reader(csv_file, delimiter=';')
        for row in reader:
            muscle_coordinates[row[0]] = row[1:]

    # load opensim model
    model = opensim.Model(model_file)
    state = model.initSystem()

    model_coordinates = {}
    for i in range(0, model.getNumCoordinates()):
        model_coordinates[model.getCoordinateSet().get(i).getName()] = i

    model_muscles = {}
    for i in range(0, model.getNumControls()):
        model_muscles[model.getMuscles().get(i).getName()] = i

    # calculate moment arm matrix (R) symbolically
    R = []
    sampling_dict = {}
    resolution = {1: 15, 2: 10, 3: 8, 4: 5, 5: 5}
    for muscle, k in tqdm(
            sorted(model_muscles.items(), key=operator.itemgetter(1))):
        # get initial state each time
        state = model.initSystem()
        coordinates = muscle_coordinates[muscle]
        N = resolution[len(coordinates)]

        # calculate moment arms for this muscle and spanning coordinates
        sampling_grid = construct_coordinate_grid(model, coordinates, N)
        moment_arm = []
        for q in sampling_grid:
            for i, coordinate in enumerate(coordinates):
                model.updCoordinateSet().get(coordinate).setValue(state, q[i])

            # model.realizePosition(state)
            tmp = []
            for coordinate in coordinates:
                coord = model.getCoordinateSet().get(coordinate)
                tmp.append(model.getMuscles()
                           .get(muscle).computeMomentArm(state, coord))

            moment_arm.append(tmp)

        moment_arm = np.array(moment_arm)
        sampling_dict[muscle] = {
            'coordinates': coordinates,
            'sampling_grid': sampling_grid,
            'moment_arm': moment_arm
        }

        # polynomial regression
        degree = 4
        muscle_moment_row = [0] * len(model_coordinates)
        for i, coordinate in enumerate(coordinates):
            coeffs, powers = multipolyfit(
                sampling_grid, moment_arm[:, i], degree, powers_out=True)
            polynomial = mk_sympy_function(coeffs, powers)
            polynomial = polynomial.subs(
                dict(
                    zip(polynomial.free_symbols,
                        [sp.Symbol(x) for x in coordinates])))
            muscle_moment_row[model_coordinates[coordinate]] = polynomial

        R.append(muscle_moment_row)

    # export data to file because the process is time consuming
    R = sp.Matrix(R)
    pickle.dump(R, file(results_dir + 'R.dat', 'w'))
    pickle.dump(sampling_dict, file(results_dir + 'sampling_dict.dat', 'w'))
    pickle.dump(model_muscles, file(results_dir + 'model_muscles.dat', 'w'))
    pickle.dump(model_coordinates, file(
        results_dir + 'model_coordinates.dat', 'w'))


def calculate_spanning_muscle_coordinates(model_file, results_dir):
    """Calculates the coordinates that are spanned by each muscle. Useful for
    reducing the required computation of the muscle moment arm matrix.

    """
    model = opensim.Model(model_file)
    state = model.initSystem()

    # construct model tree (parent body - joint - child body)
    model_tree = []
    for i in range(0, model.getJointSet().getSize()):
        joint = model.getJointSet().get(i)
        model_tree.append({
            'parent': joint.getParentName(),
            'joint': joint.getName(),
            'child': joint.getBody().getName()
        })

    ordered_body_set = []
    for i in range(0, model.getBodySet().getSize()):
        ordered_body_set.append(model.getBodySet().get(i).getName())

    # get the coordinates that are spanned by the muscles
    muscle_coordinates = {}
    for i in range(0, model.getMuscles().getSize()):
        muscle = model.getMuscles().get(i)
        path = muscle.getGeometryPath().getPathPointSet()
        muscle_bodies = []
        for j in range(0, path.getSize()):
            point = path.get(j)
            muscle_bodies.append(point.getBodyName())

        # remove duplicate bodies and sort by multibody tree order
        muscle_bodies = list(set(muscle_bodies))
        muscle_bodies = sorted(muscle_bodies,
                               key=lambda x: ordered_body_set.index(x))

        # find intermediate joints
        assert(len(muscle_bodies) > 1)
        joints = []
        find_intermediate_joints(muscle_bodies[0], muscle_bodies[-1],
                                 model_tree, joints)

        # find spanning coordinates
        muscle_coordinates[muscle.getName()] = []
        for joint in joints:
            joint = model.getJointSet().get(joint)
            for c in range(0, joint.get_CoordinateSet().getSize()):
                coordinate = joint.get_CoordinateSet().get(c)
                if coordinate.isDependent(state):
                    continue
                muscle_coordinates[muscle.getName()].append(coordinate.getName())

    # write results to file
    with open(results_dir + 'muscle_coordinates.csv', 'w') as csv_file:
        for key, values in muscle_coordinates.items():
            csv_file.write(key)
            for value in values:
                csv_file.write(';' + value)

            csv_file.write('\n')


def export_moment_arm_as_c_function(R, model_coordinates, file_name,
                                    results_dir):
    """Exports the moment arm matrix R [coordinates x muscles] as a
    callable functions of the coordinate positions.

    """
    (m, n) = R.shape
    assert(m > n)
    symbol_sybstitution = {sp.Symbol(key): sp.Symbol('q[' + str(val) + ']')
                           for key, val in model_coordinates.items()}
    RT = R.transpose().subs(symbol_sybstitution)

    # header
    with open(results_dir + file_name + '.h', 'w') as header_file:
        header_file.write('#ifndef MOMENT_ARM_H\n')
        header_file.write('#define MOMENT_ARM_H\n\n')
        header_file.write('#include "MomentArmExports.h"\n')
        header_file.write('#include <SimTKcommon.h>\n\n')
        header_file.write('#if __GNUG__\n')
        header_file.write('#define OPTIMIZATION __attribute__ ((optimize(0)))\n')
        header_file.write('#else\n#define OPTIMIZATION\n#endif\n\n')
        header_file.write('#ifdef __cplusplus\n' +
                          'extern "C" {\n')
        header_file.write('MomentArm_API ' +
                          'SimTK::Matrix calcMomentArm(const SimTK::Vector& q) ' +
                          'OPTIMIZATION;\n')
        header_file.write('}\n' +
                          '#endif\n\n')
        header_file.write('#endif')

    # source
    with open(results_dir + file_name + '.cpp', 'w') as source_file:
        source_file.write('#include "' + file_name + '.h"\n\n')
        source_file.write('using namespace SimTK;\n\n')
        source_file.write('Matrix calcMomentArm(const Vector& q) {\n')
        source_file.write('    Matrix R(' + str(n) + ', ' + str(m) + ', 0.0);\n')

        print('Exporting...')
        for i in tqdm(range(0, n)):
            for j in range(0, m):
                if RT[i, j] is sp.S.Zero:
                    continue

                source_file.write('    R[' + str(i) + '][' + str(j) + '] = ')
                source_file.write(sp.ccode(RT[i, j]))
                source_file.write(';\n')

        source_file.write('    return R;\n')
        source_file.write('}\n')


###############################################################################
# main

# def main():

# model
subject_dir = os.path.abspath('../')
model_file = os.path.join(subject_dir,
                          'model/model_generic.osim')
results_dir = os.path.join(subject_dir, 'real_time/moment_arm/')

# read opensim files
if not os.path.isfile(model_file):
    raise RuntimeError('required files do not exist')

if not os.path.isdir(results_dir):
    raise RuntimeError('required folders do not exist')

# when computed once results are stored into files and loaded with
# (pickle)
compute = True
visualize = True

if compute:
    calculate_spanning_muscle_coordinates(model_file, results_dir)
    calculate_moment_arm_symbolically(model_file, results_dir)

if visualize:
    with open(results_dir + 'R.dat', 'rb') as f_r,\
         open(results_dir + 'sampling_dict.dat', 'rb') as f_sd,\
         open(results_dir + 'model_coordinates.dat', 'rb') as f_mc,\
         open(results_dir + 'model_muscles.dat', 'rb') as f_mm:
        R = pickle.load(f_r)
        sampling_dict = pickle.load(f_sd)
        model_coordinates = pickle.load(f_mc)
        model_muscles = pickle.load(f_mm)

    # export moment arm
    export_moment_arm_as_c_function(R, model_coordinates,
                                    'MomentArm',
                                    results_dir + '/code_generation/')

    # visualize data
    with PdfPages(results_dir + 'compare_moment_arm.pdf') as pdf:
        for muscle in sampling_dict.keys():
            coordinates = sampling_dict[muscle]['coordinates']
            if len(coordinates) == 1:
                visualize_moment_arm(coordinates[0], muscle,
                                     coordinates[0],
                                     sampling_dict,
                                     model_coordinates,
                                     model_muscles, R, pdf)
            elif len(coordinates) == 2:
                visualize_moment_arm(coordinates[0], muscle, coordinates,
                                     sampling_dict, model_coordinates,
                                     model_muscles, R, pdf)
                visualize_moment_arm(coordinates[1], muscle, coordinates,
                                     sampling_dict, model_coordinates,
                                     model_muscles, R, pdf)
            else:
                print('only 2D and 3D visualization, skip: ' + muscle)
