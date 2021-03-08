# Description

This folder contains the OpenSim models, input data, analysis results, and
scripts.

- `gait1992`: template folder of a lower limb model with offline OpenSim and
  real-time analysis results. For more details, please read the readme file.
- `mobl2016`: a version of the MoBL upper limb model adapted for IMU inverse
  kinematics tracking.
- `geometry*`: folders containing geometries for the model. They are referenced
  through a symbolic link from within sub-directors. If you experience
  visualization issues with the models, please copy the content of these folders
  into the `Geometry` folder within `OPENSIM_HOME` (OpenSim environment
  variable).
- `setup.ini`: settings for testing the algorithms.
