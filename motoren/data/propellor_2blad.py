import numpy as np
import matplotlib.pyplot as plt

# Data voor propellor 2 bladed in het positieve scenario
PWM_pos_prop2 = np.array([51, 102, 153, 204, 255])
meting_pos_prop2 = np.array([11.34, 38.56, 79.39, 129.29, 185.99])

# Data voor propellor 2 bladed in het negatieve scenario
PWM_neg_prop2 = np.array([51, 102, 153, 204, 255])
meting_neg_prop2 = np.array([24.95, 54.44, 97.53, 151.97, 215.48])

# Maak de plots
plt.figure(figsize=(8, 6))

plt.plot(PWM_pos_prop2, meting_pos_prop2, color='blue',marker = 'o', label='Motor 2 (+)')
plt.plot(PWM_neg_prop2, meting_neg_prop2, color='red',marker = 'o', label='Motor 1 (+)')


plt.xlabel('PWM')
plt.ylabel('Kracht (mN)')
plt.title('Motor 1, 2 bladed propellor')
plt.legend()

plt.grid(True)
