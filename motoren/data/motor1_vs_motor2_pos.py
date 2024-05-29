import numpy as np
import matplotlib.pyplot as plt

# Data voor motor 1 in het positieve scenario
PWM_1_pos = np.array([51, 102, 153, 204, 255])
meting_1_pos = np.array([4.54, 11.34, 31.75, 63.51, 102.07])

# Data voor motor 2 in het positieve scenario
PWM_2_pos = np.array([51, 102, 153, 204, 255])
meting_2_pos = np.array([4.54, 11.34, 36.29, 68.05, 106.61])

# Maak de plots
plt.figure(figsize=(8, 6))

plt.plot(PWM_2_pos, meting_2_pos, color='blue',marker = 'o', label='Motor 2 (+)')
plt.plot(PWM_1_pos, meting_1_pos, color='red',marker = 'o', label='Motor 1 (+)')


plt.xlabel('PWM')
plt.ylabel('Kracht (mN)')
plt.title('Motor 1 vs Motor 2 (+)')
plt.legend()

plt.grid(True)
