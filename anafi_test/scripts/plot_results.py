import matplotlib.pyplot as plt
import numpy as np

import pandas as pd
_dir = "/home/anafi/anafi_ws/src/anafi_ros/anafi_test/results/"
f_name = _dir+"test_1659621568.6070068.csv"
df = pd.read_csv(f_name, delimiter=",")
# print(df.head(5))

# print(df.columns)

pitch = -df['pitch']
pitch = pitch[100:2000]
print(np.min(pitch))

pitch = pitch[pitch > 0]
print(np.min(pitch))
pitch = pitch[pitch > 5]
print(np.mean(pitch))
plt.plot(pitch)
plt.show()
