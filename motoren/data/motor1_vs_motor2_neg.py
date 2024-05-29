import numpy as np
import matplotlib.pyplot as plt

# Data voor motor 1 in het negatieve scenario
PWM_1_neg = np.array([51, 102, 153, 204, 255])
meting_1_neg = np.array([18.15, 31.76, 63.51, 113.41, 165.58])

# Data voor motor 2 in het negatieve scenario
PWM_2_neg = np.array([51, 102, 153, 204, 255])
meting_2_neg = np.array([13.61, 31.75, 65.78, 115.68, 167.85])

# Maak de plots
plt.figure(figsize=(8, 6))

plt.plot(PWM_2_neg, meting_2_neg, color='blue',marker = 'o', label='Motor 2 (-)')
plt.plot(PWM_1_neg, meting_1_neg, color='red',marker = 'o', label='Motor 1 (-)')

plt.xlabel('PWM')
plt.ylabel('Kracht (mN)')
plt.title('Motor 1 vs Motor 2 (-)')
plt.legend()

plt.grid(True)
