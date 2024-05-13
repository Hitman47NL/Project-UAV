import numpy as np
import matplotlib.pyplot as plt

# Data voor propellor 6 bladed in het positieve scenario
PWM_pos_prop6 = np.array([51, 102, 153, 204, 255])
meting_pos_prop6 = np.array([])

# Data voor propellor 6 bladed in het negatieve scenario
PWM_neg_prop6 = np.array([51, 102, 153, 204, 255])
meting_neg_prop6 = np.array([])

# Maak de plots
plt.figure(figsize=(8, 6))

plt.plot(PWM_pos_prop6, meting_pos_prop6, color='blue',marker = 'o', label='Motor 2 (+)')
plt.plot(PWM_neg_prop6, meting_neg_prop6, color='red',marker = 'o', label='Motor 1 (+)')


plt.xlabel('PWM')
plt.ylabel('Kracht (N)')
plt.title('Motor 1, 6 bladed propellor')
plt.legend()

plt.grid(True)
