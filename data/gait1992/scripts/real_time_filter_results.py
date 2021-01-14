# Identification of optimal filter parameters for memory and delay.
#
# author: Dimitar Stanev <jimstanev@gmail.com>
# %%
import os
import numpy as np
import matplotlib
matplotlib.rcParams.update({'font.size': 14})
import matplotlib.pyplot as plt
import seaborn as sns

# %%

subject_dir = os.path.abspath('../')
output_dir = os.path.join(subject_dir, 'real_time/filtering/fig')


# %%
# memory

memory = [20, 26, 30, 35, 40, 45, 50, 55, 60, 65, 70]
q_rmse_m = [0.171, 0.156, 0.126, 0.107, 0.109, 0.113, 0.12, 0.126, 0.133, 0.134, 0.131]
q_rmse_s = [0.155, 0.138, 0.103, 0.086, 0.1, 0.109, 0.121, 0.131, 0.143, 0.145, 1.139]
u_rmse_m = [3.429, 3.21, 2.913, 2.606, 2.3, 2.189, 2.121, 2.131, 2.223, 2.31, 2.366]
u_rmse_s = [2.863, 2.652, 2.363, 2.079, 1.807, 1.713, 1.658, 1.67, 1.767, 1.854, 1.912]
a_rmse_m = [132.904, 123.876, 120.156, 119.129, 120.658, 123.541, 127.936, 130.531, 131.356, 131.49, 133.442]
a_rmse_s = [142.034, 137.633, 135.613, 133.315, 132.548, 135.543, 140.462, 142.746, 142.132, 141.106, 142.478]

plt.figure()
ax1 = sns.regplot(x=memory, y=q_rmse_m, order=3)
# ax1.set_title('generalized coordinates')
ax1.set_ylabel('coordinates RMSE (deg)')
ax1.set_xlabel('memory (samples)')
plt.tight_layout()
plt.savefig(output_dir + '/coordinates_memory.pdf')

plt.figure()
ax2 = sns.regplot(x=memory, y=u_rmse_m, order=2)
# ax2.set_title('generalized speeds')
ax2.set_ylabel('speeds RMSE (deg / s)')
ax2.set_xlabel('memory (samples)')
plt.tight_layout()
plt.savefig(output_dir + '/speeds_memory.pdf')

plt.figure()
ax3 = sns.regplot(x=memory, y=a_rmse_m, order=3)
# ax3.set_title('generalized accelerations')
ax3.set_ylabel('accelerations RMSE (deg / s^2)')
ax3.set_xlabel('memory (samples)')
plt.tight_layout()
plt.savefig(output_dir + '/accelerations_memory.pdf')

# %%
# delay

delay = [10, 12, 14, 16, 18, 19, 25]
q_rmse_m = [0.107, 0.107, 0.108, 0.107, 0.107, 0.107, 0.105]
q_rmse_s = [0.086, 0.086, 0.087, 0.078, 0.086, 0.086, 0.084]
u_rmse_m = [2.606, 2.329, 2.283, 2.283, 2.284, 2.283, 2.847]
u_rmse_s = [2.079, 1.845, 1.807, 1.809, 1.811, 1.809, 2.357]
a_rmse_m = [119.129, 113.01, 108.014, 106.971, 106.886, 107.077, 114.104]
a_rmse_s = [133.315, 130.824, 129.286, 129.572, 129.86, 130.129, 130.692]

plt.figure()
ax1 = sns.regplot(x=delay, y=q_rmse_m)
# ax1.set_title('generalized coordinates')
ax1.set_ylabel('coordinates RMSE (deg)')
ax1.set_xlabel('delay (samples)')
plt.tight_layout()
plt.savefig(output_dir + '/coordinates_delay.pdf')

plt.figure()
ax2 = sns.regplot(x=delay, y=u_rmse_m, order=2)
# ax2.set_title('generalized speeds')
ax2.set_ylabel('speeds RMSE (deg / s)')
ax2.set_xlabel('delay (samples)')
plt.tight_layout()
plt.savefig(output_dir + '/speeds_delay.pdf')

plt.figure()
ax3 = sns.regplot(x=delay, y=a_rmse_m, order=2)
# ax3.set_title('generalized accelerations')
ax3.set_ylabel('accelerations RMSE (deg / s^2)')
ax3.set_xlabel('delay (samples)')
plt.tight_layout()
plt.savefig(output_dir + '/accelerations_delay.pdf')

plt.show()

# %%
