import numpy as np
import matplotlib.pyplot as plt

# Data voor propellor 6 bladed in het positieve scenario
PWM_pos_prop6 = np.array([51, 102, 153, 204, 255])
meting_pos_prop6 = np.array([15.88, 65.78, 115.68, 179.19, 238.16])

# Data voor propellor 6 bladed in het negatieve scenario
PWM_neg_prop6 = np.array([51, 102, 153, 204, 255])
meting_neg_prop6 = np.array([31.75, 72.58, 122.48, 90.53, 267.65])

# Maak de plots
plt.figure(figsize=(8, 6))

plt.plot(PWM_pos_prop6, meting_pos_prop6, color='blue',marker = 'o', label='Motor 2 (+)')
plt.plot(PWM_neg_prop6, meting_neg_prop6, color='red',marker = 'o', label='Motor 1 (+)')


plt.xlabel('PWM')
plt.ylabel('Kracht (N)')
plt.title('Motor 1, 6 bladed propellor')
plt.legend()

plt.grid(True)
