import numpy as np
import matplotlib.pyplot as plt

# Data voor motor 2 in het positieve scenario
voltage_2_pos = np.array([1, 2, 3, 4, 5, 6])
meting_2_pos = np.array([6.80, 13.61, 29.49, 54.44, 81.66, 113.41])

# Data voor motor 2 in het negatieve scenario
voltage_2_neg = np.array([1, 2, 3, 4, 5, 6])
meting_2_neg = np.array([4.54, 20.41, 43.09, 77.12, 115.68, 163.31])

# Maak de plots
plt.figure(figsize=(8, 6))

plt.plot(voltage_2_pos, meting_2_pos, color='blue',marker = 'o', label='Motor 2 (+)')
plt.plot(voltage_2_neg, meting_2_neg, color='red',marker = 'o', label='Motor 2 (-)')


plt.xlabel('Voltage (V)')
plt.ylabel('Kracht (N)')
plt.title('Metingen van Motor 2')
plt.legend()

plt.grid(True)

