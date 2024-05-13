import numpy as np
import matplotlib.pyplot as plt

# Data voor motor 2 in het positieve scenario
voltage_2_pos = np.array([1, 2, 3])
meting_2_pos = np.array([20.41, 63.51, 90.73])

# Data voor motor 2 in het negatieve scenario
voltage_2_neg = np.array([1, 2, 3])
meting_2_neg = np.array([29.49, 99.80, 136.09])

# Maak de plots
plt.figure(figsize=(8, 6))

plt.plot(voltage_2_pos, meting_2_pos, color='blue', marker = 'o', label='Motor 2 (+)')
plt.plot(voltage_2_neg, meting_2_neg, color='red', marker = 'o', label='Motor 2 (-)')

plt.xlabel('Voltage (V)')
plt.ylabel('Kracht (N)')
plt.title('Metingen van kleine motor')
plt.legend()

plt.grid(True)
