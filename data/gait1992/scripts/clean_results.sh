#!/bin/bash
cd ..
find . -name '*.log' -delete
# find . -name '*.sto' -delete
# find . -name '*.pdf' -delete
find . -name '*adjusted*' -delete
rm scale/model_scaled.osim
rm scale/model_static.mot
rm scale/model_scale_set_applied.xml
rm inverse_kinematics/task_InverseKinematics.mot
rm residual_reduction_algorithm/task_avgResiduals.txt
rm residual_reduction_algorithm/task_controls.xml
rm static_optimization/task_StaticOptimization_controls.xml
rm computed_muscle_controls/task_controls.xml
cd scripts/
