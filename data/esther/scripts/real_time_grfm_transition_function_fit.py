import os
import numpy as np
from scipy.optimize import curve_fit
from utils import read_from_storage, rmse_metric, plot_sto_file, annotate_plot
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

time = np.array([0., 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1,
                 0.11, 0.12, 0.13, 0.14, 0.15, 0.16, 0.17, 0.18, 0.19, 0.2, 0.21,
                 0.22, 0.23])

Fx_total = np.array([98.36042366,
                     106.3470253,
                     113.3207813,
                     118.7155953,
                     121.9685473,
                     122.5519191,
                     120.0109494,
                     114.0473392,
                     104.5893112,
                     91.7931287,
                     76.05978827,
                     57.97448055,
                     38.3127292,
                     17.95726164,
                     -2.181689121,
                     -21.23364525,
                     -38.43411956,
                     -53.18439415,
                     -65.0886592,
                     -73.95968297,
                     -79.8112289,
                     -82.83804141,
                     -83.38353754,
                     -81.88924417
                     ])

Fx_trail = np.array([2.130849378,
                     2.218921785,
                     1.732092852,
                     0.390530831,
                     -2.054723707,
                     -5.79559043,
                     -10.9438498,
                     -17.49544923,
                     -25.31017618,
                     -34.1243665,
                     -43.57793076,
                     -53.24146566,
                     -62.65263776,
                     -71.36486588,
                     -78.99078787,
                     -85.22854578,
                     -89.87875316,
                     -92.85613023,
                     -94.18530607,
                     -93.97742019,
                     -92.40880649,
                     -89.70223932,
                     -86.10892278,
                     -81.88339106
                     ])

ydata = np.divide(Fx_trail, np.abs(Fx_total[0]))


def func(t, B, M, m1, m2, c):
    return 1 / (1.0 + np.exp(B * (t - m1))) + \
        M * np.exp(-np.power((t - m2), 2) / (2 * np.power(c, 2)))


params, cov = curve_fit(func, time, ydata, method='lm')

print(params)

plt.plot(time, ydata)
plt.plot(time, func(time, *params))
plt.show()
