# Utilities for converting .c3d files to OpenSim marker.trc and ground
# reaction forces grf.mot.
#
# author: Dimitar Stanev <jimstanev@gmail.com>
# contributor: Celine Provins
##
import os
import opensim
import numpy as np
import matplotlib.pyplot as plt
from scipy import linalg
import pandas as pd


def visu_matrix(mat, c):
    '''Visualize a column of a matrix

    Parameters
    ----------
    mat : Matrix of type OpenSim.Simbody.Matrix
    c : int corresponding to the index of the column you want to visualize

    '''
    mat_np = np.zeros((mat.nrow(), mat.ncol()))
    for j in range(mat.ncol()):
        for i in range(mat.nrow()):
            mat_np[i, j] = mat.get(i, j)
            plt.plot(mat_np[:, c])
            plt.show()


def visu_vec(v):
    ''' Visualize a vector

    Parameters
    ----------
    vec : Vec of type opensim.simbody.VectorViewVec3
    '''
    r = v.nrow()
    v_np = np.zeros((r, 3))
    for i in range(r):
        v_np[i, 0] = v[i][0]  # extract x component at each time step
        v_np[i, 1] = v[i][1]
        v_np[i, 2] = v[i][2]

    # print(v_np[1600:1900, 1])
    plt.plot(v_np)
    plt.show()


def rotate(table, axis, deg):
    '''Rotate OpenSim.Table entries using an axis and angle.

    Parameters
    ----------
    table: OpenSim.common.TimeSeriesTableVec3

    axis: 3x1 vector

    deg: angle in degrees

    '''
    R = opensim.Rotation(np.deg2rad(deg),
                         opensim.Vec3(axis[0], axis[1], axis[2]))
    for i in range(table.getNumRows()):
        vec = table.getRowAtIndex(i)
        vec_rotated = R.multiply(vec)
        table.setRowAtIndex(i, vec_rotated)


def mm_to_m(table, label):
    '''Adjust the current units (mm) for OpenSim convention (m)

    Parameters
    ----------
    label: string containing the name of the column you want to convert

    '''
    c = table.updDependentColumn(label)
    for i in range(c.size()):
        c[i] = opensim.Vec3(c[i][0] * 0.001, c[i][1] * 0.001, c[i][2] * 0.001)


def mirror_z(table, label):
    '''Mirror the z-component of the vector

    Parameters
    ----------
    label: string containing the name of the column you want to convert

    '''
    c = table.updDependentColumn(label)
    for i in range(c.size()):
        c[i] = opensim.Vec3(c[i][0], c[i][1], -c[i][2])


def lowess_bell_shape_kern(t, v, tau=.0005, output_dir='.'):
    '''lowess_bell_shape_kern(t, v, tau = .005) -> vest Locally weighted
    regression: fits a nonparametric regression curve to a
    scatterplot.  The arrays t and v contain an equal number of
    elements; each pair (t[i], v[i,j]) defines a data point in the
    scatterplot. Depending on j, this corresponds to the x,y or z
    column of the matrix v. The function returns the estimated
    (smooth) values of each columns of y in a matrix.  The kernel
    function is the bell shaped function with parameter tau. Larger
    tau will result in a smoother curve.

    '''
    r = len(t)

    # convert tuple into np.array
    t_np = np.zeros(r)
    for i in range(r):
        t_np[i] = t[i]

    # convert Vec3 into np.array
    v_np = np.zeros((r, 3))
    for i in range(r):
        v_np[i, 0] = v[i][0]  # extract x component at each time step
        v_np[i, 1] = v[i][1]
        v_np[i, 2] = v[i][2]

    # interpolate to replace NaN
    v_int = np.zeros((r, 3))
    for j in range(3):
        v_pd = pd.Series(v_np[:, j])
        v_pd = v_pd.interpolate(limit_direction="both", kind="cubic")
        v_int[:, j] = v_pd.to_numpy()

    # initializing all weights from the bell shape kernel function
    for j in range(3):
        w = [np.exp(- (t_np - t_np[i])**2/(2*tau)) for i in range(r)]

    # looping through all v-points
    vest = np.zeros((r, 3))
    for j in range(3):
        for i in range(r):
            weights = w[i]
            b = np.array([np.sum(weights * v_int[:, j]),
                          np.sum(weights * v_int[:, j] * t_np)])
            A = np.array([[np.sum(weights), np.sum(weights * t_np)],
                          [np.sum(weights * t_np), np.sum(weights * t_np * t_np)]])
            theta = linalg.solve(A, b)
            vest[i, j] = theta[0] + theta[1] * t_np[i]

    plt.figure()
    plt.plot(v_np[:, 0], label='raw')
    plt.ylabel(label)
    plt.xlabel('sample')
    plt.plot(vest[:, 0], label='filtered')
    plt.legend()
    plt.savefig(output_dir + '/{}_x.pdf'.format(label))

    plt.figure()
    plt.plot(v_np[:, 1], label='raw')
    plt.ylabel(label)
    plt.xlabel('sample')
    plt.plot(vest[:, 1], label='filtered')
    plt.legend()
    plt.savefig(output_dir + '/{}_y.pdf'.format(label))

    plt.figure()
    plt.plot(v_np[:, 2], label='raw')
    plt.ylabel(label)
    plt.xlabel('sample')
    plt.plot(vest[:, 2], label='filtered')
    plt.legend()
    plt.savefig(output_dir + '/{}_z.pdf'.format(label))
    plt.close('all')

    return vest

##
# specify the .c3d file

static_file = 'static.c3d'
walk_file = 'walk.c3d'
subject_dir = os.path.abspath('../experimental_data/')
c3d_dir = subject_dir
output_dir = subject_dir

adapter = opensim.C3DFileAdapter()
trc_adapter = opensim.TRCFileAdapter()
sto_adapter = opensim.STOFileAdapter()

##
# extract data for static

# get markers
static = adapter.read(os.path.join(c3d_dir, static_file), 1)
markers_static = static['markers']

# process markers_walk
rotate(markers_static, [1, 0, 0], -90)
# rotate(markers_static, [0, 1, 0], 180)
# rotate(markers_static, [0, 1, 0], -90)
trc_adapter.write(markers_static, os.path.join(output_dir, 'static.trc'))

##
# extract data for walk

# get markers and forces
walk = adapter.read(os.path.join(c3d_dir, walk_file), 1)
markers_walk = walk['markers']
forces_walk = walk['forces']

# process markers_walk
rotate(markers_walk, [1, 0, 0], -90)
# rotate(markers_walk, [0, 1, 0], 180)
# rotate(markers_walk, [0, 1, 0], -90)
trc_adapter = opensim.TRCFileAdapter()
trc_adapter.write(markers_walk, os.path.join(output_dir, 'walk.trc'))

# process forces
rotate(forces_walk, [1, 0, 0], 90)
rotate(forces_walk, [0, 1, 0], 180)
rotate(forces_walk, [0, 0, 1], 180)
mirror_z(forces_walk, 'f1')
mirror_z(forces_walk, 'f2')

# conversion of unit
mm_to_m(forces_walk, 'p1')
mm_to_m(forces_walk, 'p2')
mm_to_m(forces_walk, 'm1')
mm_to_m(forces_walk, 'm2')

# low-pass filter forces, moments and torques
t = forces_walk.getIndependentColumn()
labels = forces_walk.getColumnLabels()
list_mat = list()
for label in labels:
    f = forces_walk.getDependentColumn(label)
    list_mat.append(lowess_bell_shape_kern(t, f, output_dir=output_dir))

forces_walk_np = np.array(list_mat)
# construct the matrix of the forces (forces, moments, torques / right
# and left) (type opensim.Matrix)
forces_walk_mat = opensim.Matrix(len(t), 18)
for n in range(6):
    for j in range(3):
        for i in range(len(t)):
            forces_walk_mat.set(i, 3 * n + j, forces_walk_np[n, i, j])

# export forces
labels_list = ['ground_force_vx', 'ground_force_vy', 'ground_force_vz',
               'ground_force_px', 'ground_force_py', 'ground_force_pz',
               'ground_torque_x', 'ground_torque_y', 'ground_torque_z',
               '1_ground_force_vx', '1_ground_force_vy', '1_ground_force_vz',
               '1_ground_force_px', '1_ground_force_py', '1_ground_force_pz',
               '1_ground_torque_x', '1_ground_torque_y', '1_ground_torque_z']
table = opensim.TimeSeriesTable(t, forces_walk_mat, labels_list)
sto_adapter.write(table, os.path.join(output_dir, 'walk_grf.mot'))
