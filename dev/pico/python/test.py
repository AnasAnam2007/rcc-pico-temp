import matplotlib
import matplotlib.pyplot as plt
import numpy as np
t = np.arange(-5, 5, 0.001)
x = 2*t
y = np.sin(t)

fig, ax = plt.subplots()
ax.plot(t, x, 'b', label = 'example function 1')
ax.plot(t, y, 'r', label = 'example function 2')
ax.set(xlabel= 'time', ylabel= 'x, y', title= 'Example Plots')
ax.legend(loc = 'upper right')
ax.grid()
plt.show()
