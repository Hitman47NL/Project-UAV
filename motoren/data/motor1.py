import numpy as np
import matplotlib.pyplot as plt

# Data voor motor 2 in het positieve scenario
voltage_2_pos = np.array([1, 2, 3, 4, 5, 6])
meting_2_pos = np.array([6.8, 20.41, 36.30, 54.44, 86.20, 117.95])

# Data voor motor 2 in het negatieve scenario
voltage_2_neg = np.array([1, 2, 3, 4, 5, 6])
meting_2_neg = np.array([6.8, 20.41, 49.9, 83.92, 124.75, 176.92])

# Bereken de helling van de regressielijnen voor elk scenario
helling_2_pos = np.polyfit(voltage_2_pos, meting_2_pos, 1)[0]
helling_2_neg = np.polyfit(voltage_2_neg, meting_2_neg, 1)[0]

# Maak de plots
plt.figure(figsize=(8, 6))

plt.scatter(voltage_2_pos, meting_2_pos, color='blue', label='Motor 2 (+)')
plt.plot(voltage_2_pos, np.polyval(np.polyfit(voltage_2_pos, meting_2_pos, 1), voltage_2_pos), color='blue', linestyle='--', label=f'Regressielijn (+): helling={helling_2_pos:.2f}')

plt.scatter(voltage_2_neg, meting_2_neg, color='red', label='Motor 2 (-)')
plt.plot(voltage_2_neg, np.polyval(np.polyfit(voltage_2_neg, meting_2_neg, 1), voltage_2_neg), color='red', linestyle='--', label=f'Regressielijn (-): helling={helling_2_neg:.2f}')

plt.xlabel('Voltage (V)')
plt.ylabel('Meting (g)')
plt.title('Metingen van Motor 1')
plt.legend()

plt.grid(True)

