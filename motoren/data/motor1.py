import numpy as np
import matplotlib.pyplot as plt

# Data voor motor 2 in het positieve scenario
voltage_2_pos = np.array([1, 2, 3, 4, 5, 6])
meting_2_pos = np.array([6.8, 18.15, 34.02, 56.71, 83.92, 117.95])

# Data voor motor 2 in het negatieve scenario
voltage_2_neg = np.array([1, 2, 3, 4, 5, 6])
meting_2_neg = np.array([4.54, 15.88, 40.83, 72.58, 117.95, 163.31])

# Maak de plots
plt.figure(figsize=(8, 6))

plt.plot(voltage_2_pos, meting_2_pos, color='blue', label='Motor 2 (+)')
plt.plot(voltage_2_neg, meting_2_neg, color='red', label='Motor 2 (-)')


plt.xlabel('Voltage (V)')
plt.ylabel('Kracht (N)')
plt.title('Metingen van Motor 1')
plt.legend()

plt.grid(True)

