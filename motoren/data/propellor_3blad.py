import numpy as np
import matplotlib.pyplot as plt

# Data voor propellor 3 bladed in het positieve scenario
PWM_pos_prop3 = np.array([51, 102, 153, 204, 255])
meting_pos_prop3 = np.array([18.15, 45.36, 83.92, 138.36, 204.14])

# Data voor propellor 3 bladed in het negatieve scenario
PWM_neg_prop3 = np.array([51, 102, 153, 204, 255])
meting_neg_prop3 = np.array([13.61, 58.97, 111.14, 172.38, 244.97])

# Maak de plots
plt.figure(figsize=(8, 6))

plt.plot(PWM_pos_prop3, meting_pos_prop3, color='blue',marker = 'o', label='Motor 2 (+)')
plt.plot(PWM_neg_prop3, meting_neg_prop3, color='red',marker = 'o', label='Motor 1 (+)')


plt.xlabel('PWM')
plt.ylabel('Kracht (mN)')
plt.title('Motor 1, 3 bladed propellor')
plt.legend()

plt.grid(True)
