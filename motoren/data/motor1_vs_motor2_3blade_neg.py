import numpy as np
import matplotlib.pyplot as plt

# Data voor motor 1 in het negatieve scenario
PWM_1_neg = np.array([51, 102, 153, 204, 255])
meting_1_neg = np.array([13.61, 58.97, 111.14, 172.38, 244.97])

# Data voor motor 2 in het negatieve scenario
PWM_2_neg = np.array([51, 102, 153, 204, 255])
meting_2_neg = np.array([10.21, 58.911, 115.107, 175.82, 248.32])

# Maak de plots
plt.figure(figsize=(8, 6))

plt.plot(PWM_2_neg, meting_2_neg, color='blue',marker = 'o', label='Motor 2 (-)')
plt.plot(PWM_1_neg, meting_1_neg, color='red',marker = 'o', label='Motor 1 (-)')

plt.xlabel('PWM')
plt.ylabel('Kracht (mN)')
plt.title('Motor 1 vs Motor 2 (-), 3 blade propellor')
plt.legend()

plt.grid(True)
