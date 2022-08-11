import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit
import pandas as pd


def func(x, a, b, c, d):
    
    return a*x**3 + b*x**2 + c*x + d


_dir = "/home/anafi/anafi_ws/src/anafi_ros/anafi_test/results/7secs_25%/"
f_name = _dir+"test_1660219998.3515844.csv"
df = pd.read_csv(f_name, delimiter=",")
# # print(df.head(5))

# # print(df.columns)

pitch = df['pitch'] #put negative when backwards data
# pitch = pitch[600:1500]
# # print(np.min(pitch))

pitch = pitch[pitch > 5]
# # print(np.min(pitch))
# # pitch = pitch[pitch > 11]
# print(np.mean(pitch))
plt.plot(pitch)
plt.show()

# ## Curve fitting
data_range = len(pitch)
xdata = np.linspace(0, 10, data_range)
plt.plot(xdata, pitch)

popt, pcov = curve_fit(func, xdata, pitch)
print(popt)
red_line = func(xdata, *popt)
print(np.max(red_line))
print(np.mean(red_line))
print(np.min(red_line))
plt.plot(xdata, func(xdata, *popt), 'r-',

         label='fit: a=%5.3f, b=%5.3f, c=%5.3f, d=%5.3f' % tuple(popt))

plt.show()