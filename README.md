[![Build Status](https://travis-ci.com/mitkof6/OpenSimRT.svg?token=6jXyQbpyyXVTJGTxHvxD&branch=master)](https://travis-ci.com/mitkof6/OpenSimRT)

# Real-Time Musculoskeletal Kinematics and Dynamics Analysis Using Marker- and IMU-Based Solutions in Rehabilitation

<p align="center"><iframe width="560" height="315" src="https://mitkof6.gitlab.io/personal-site/publications/sensors2021/real_time_framework_video.mp4" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe></p>

This study aims to explore the possibility of estimating a multitude of
kinematic and dynamic quantities using subject-specific musculoskeletal models
in real-time. The framework was designed to operate with marker-based and
inertial measurement units enabling extensions far beyond dedicated motion
capture laboratories. We present the technical details for calculating the
kinematics, generalized forces, muscle forces, joint reaction loads, and
predicting ground reaction wrenches during walking. Emphasis was given to reduce
computational latency while maintaining accuracy as compared to the offline
counterpart. Notably, we highlight the influence of adequate filtering and
differentiation under noisy conditions and its importance for consequent dynamic
calculations. Real-time estimates of the joint moments, muscle forces, and
reaction loads closely resemble OpenSim’s offline analyses. Model-based
estimation of ground reaction wrenches demonstrates that even a small error can
negatively affect other estimated quantities. An application of the developed
system is demonstrated in the context of rehabilitation and gait retraining. We
expect that such a system will find numerous applications in laboratory settings
and outdoor conditions with the advent of predicting or sensing environment
interactions. Therefore, we hope that this open-source framework will be a
significant milestone for solving this grand challenge.

Relative projects:

- [RTOSIM](https://github.com/RealTimeBiomechanics/rtosim)
- [CEINMS](https://simtk.org/projects/ceinms/)
- [Motekmedical HBM](https://www.motekmedical.com/software/hbm/)

# Organization


# Dependencies

This project depends on several libraries:

- [opensim-core](https://github.com/opensim-org/opensim-core) and we have
  tested with [branch](https://github.com/mitkof6/opensim-core/tree/bindings_timestepper).
- If you plan to use the `Vicon` module please use
  [ViconDataStreamSDK_1.7.1](https://www.vicon.com/software/datastream-sdk/?section=downloads).
- If you plan to use the `IMU` module that uses [NGIMU x-io Technologies
   Limited](https://x-io.co.uk/ngimu/) please build the
   [oscpack](https://github.com/mitkof6/oscpack).

For more information on how to build and set up these dependencies, please look
at the continuous integration scripts. To run the tests and examples that use
files as inputs instead of streams, you will only need `opensim-core`.

# Acknowledge

If you find this useful you can acknowledge it as follows:

```bibtex
@Article{s21051804,
AUTHOR = {Stanev, Dimitar and Filip, Konstantinos and Bitzas, Dimitrios and Zouras, Sokratis and Giarmatzis, Georgios and Tsaopoulos, Dimitrios and Moustakas, Konstantinos},
TITLE = {Real-Time Musculoskeletal Kinematics and Dynamics Analysis Using Marker- and IMU-Based Solutions in Rehabilitation},
JOURNAL = {Sensors},
VOLUME = {21},
YEAR = {2021},
NUMBER = {5},
ARTICLE-NUMBER = {1804},
URL = {https://www.mdpi.com/1424-8220/21/5/1804},
ISSN = {1424-8220},
ABSTRACT = {This study aims to explore the possibility of estimating a multitude of kinematic and dynamic quantities using subject-specific musculoskeletal models in real-time. The framework was designed to operate with marker-based and inertial measurement units enabling extensions far beyond dedicated motion capture laboratories. We present the technical details for calculating the kinematics, generalized forces, muscle forces, joint reaction loads, and predicting ground reaction wrenches during walking. Emphasis was given to reduce computational latency while maintaining accuracy as compared to the offline counterpart. Notably, we highlight the influence of adequate filtering and differentiation under noisy conditions and its importance for consequent dynamic calculations. Real-time estimates of the joint moments, muscle forces, and reaction loads closely resemble OpenSim’s offline analyses. Model-based estimation of ground reaction wrenches demonstrates that even a small error can negatively affect other estimated quantities. An application of the developed system is demonstrated in the context of rehabilitation and gait retraining. We expect that such a system will find numerous applications in laboratory settings and outdoor conditions with the advent of predicting or sensing environment interactions. Therefore, we hope that this open-source framework will be a significant milestone for solving this grand challenge.},
DOI = {10.3390/s21051804}
}
```
