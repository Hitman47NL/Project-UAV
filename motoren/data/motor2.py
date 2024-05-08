import numpy as np
import matplotlib.pyplot as plt

# Data voor motor 2 in het positieve scenario
voltage_2_pos = np.array([1, 2, 3, 4, 5, 6])
meting_2_pos = np.array([6.80, 18.16, 36.30, 56.7, 95.26, 127.02])

# Data voor motor 2 in het negatieve scenario
voltage_2_neg = np.array([1, 2, 3, 4, 5, 6])
meting_2_neg = np.array([6.8, 18.16, 43.1, 74.85, 122.5, 167.85])

# Maak de plots
plt.figure(figsize=(8, 6))

plt.plot(voltage_2_pos, meting_2_pos, color='blue', label='Motor 2 (+)')
plt.plot(voltage_2_neg, meting_2_neg, color='red', label='Motor 2 (-)')


plt.xlabel('Voltage (V)')
plt.ylabel('Meting (g)')
plt.title('Metingen van Motor 2')
plt.legend()

plt.grid(True)

