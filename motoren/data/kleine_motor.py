import numpy as np
import matplotlib.pyplot as plt

# Data voor motor 2 in het positieve scenario
voltage_2_pos = np.array([1, 2, 3])
meting_2_pos = np.array([11, 32, 51])

# Data voor motor 2 in het negatieve scenario
voltage_2_neg = np.array([-1, -2, -3])
meting_2_neg = np.array([-13, -41, -63])

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
plt.title('Metingen van kleine motor')
plt.legend()

plt.grid(True)
plt.show()
