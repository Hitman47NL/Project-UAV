import numpy as np
import matplotlib.pyplot as plt

# Data voor motor 1 in het positieve scenario
PWM_1_neg = np.array([51, 102, 153, 204, 255])
meting_1_neg = np.array([18.15, 45.36, 83.92, 138.36, 204.14])

# Data voor motor 2 in het positieve scenario
PWM_2_neg = np.array([51, 102, 153, 204, 255])
meting_2_neg = np.array([18.15, 45.36, 95.91, 148.18, 213.122])

# Maak de plots
plt.figure(figsize=(8, 6))

plt.plot(PWM_2_neg, meting_2_neg, color='blue',marker = 'o', label='Motor 2 (-)')
plt.plot(PWM_1_neg, meting_1_neg, color='red',marker = 'o', label='Motor 1 (-)')

plt.xlabel('PWM')
plt.ylabel('Kracht (mN)')
plt.title('Motor 1 vs Motor 2, (+), 3 blade propellor')
plt.legend()

plt.grid(True)
