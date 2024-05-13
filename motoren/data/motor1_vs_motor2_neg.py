import numpy as np
import matplotlib.pyplot as plt

# Data voor motor 1 in het negatieve scenario
voltage_1_neg = np.array([1, 2, 3, 4, 5, 6])
meting_1_neg = np.array([6.8, 20.41, 49.90, 83.92, 124.75, 176.92])

# Data voor motor 2 in het negatieve scenario
voltage_2_neg = np.array([1, 2, 3, 4, 5, 6])
meting_2_neg = np.array([6.8, 18.16, 43.1, 74.85, 122.50, 167.85])

# Maak de plots
plt.figure(figsize=(8, 6))

plt.plot(voltage_2_neg, meting_2_neg, color='blue', label='Motor 2 (-)')
plt.plot(voltage_1_neg, meting_1_neg, color='red', label='Motor 1 (-)')

plt.xlabel('Voltage (V)')
plt.ylabel('Kracht (N)')
plt.title('Motor 1 vs Motor 2 (-)')
plt.legend()

plt.grid(True)
