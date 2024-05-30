import numpy as np
import matplotlib.pyplot as plt

# Import and use the code from the 'data' folder
import data.Brug_UAV_GrP_M1 as motor1GP
import data.Brug_UAV_GrP_M2 as motor2GP
import data.Brug_UAV_StanP_M1 as motor1SP
import data.Brug_UAV_StanP_M2 as motor2SP

# Create subplots
fig, axs = plt.subplots(1, 4, figsize=(16, 6))

# Plot data for Groene propellor motor 1
axs[0].plot(motor1GP.PWM_UAVBGP1_pos, motor1GP.meting_UAVBGP1_pos, marker= 'o', label='Motor 1 [+]', color = 'magenta')
axs[0].plot(motor1GP.PWM_UAVBGP1_neg, motor1GP.meting_UAVBGP1_neg, marker= 'o', label='Motor 1 [-]', color = 'lime')
axs[0].set_xlabel('PWM')
axs[0].set_ylabel('Kracht [mN]')
axs[0].set_title('GP Motor 1')
axs[0].grid(True)
axs[0].legend()

# Plot data for Groene propellor motor 2
axs[1].plot(motor2GP.PWM_UAVBGP2_pos, motor2GP.meting_UAVBGP2_pos, marker= 'o', label='Motor 2 [+]', color = 'magenta')
axs[1].plot(motor2GP.PWM_UAVBGP2_neg, motor2GP.meting_UAVBGP2_neg, marker= 'o', label='Motor 2 [-]', color = 'lime')
axs[1].set_xlabel('PWM')
axs[1].set_ylabel('Kracht [mN]')
axs[1].set_title('GP Motor 2')
axs[1].grid(True)
axs[1].legend()

# Plot data for standaard propellor motor 1
axs[2].plot(motor1SP.PWM_UAVBSP1_pos, motor1SP.meting_UAVBSP1_pos, marker= 'o', label='Kleine Motor [+]', color = 'magenta')
axs[2].plot(motor1SP.PWM_UAVBSP1_neg, motor1SP.meting_UAVBSP1_neg, marker= 'o', label='Kleine Motor [-]', color = 'lime')
axs[2].set_xlabel('PWM')
axs[2].set_ylabel('Kracht [mN]')
axs[2].set_title('SP Motor 1')
axs[2].grid(True)
axs[2].legend()

# Plot data for standaar propellor motor 2
axs[2].plot(motor2SP.PWM_UAVBSP2_pos, motor2SP.meting_UAVBSP2_pos, marker= 'o', label='Kleine Motor [+]', color = 'magenta')
axs[2].plot(motor2SP.PWM_UAVBSP2_neg, motor2SP.meting_UAVBSP2_neg, marker= 'o', label='Kleine Motor [-]', color = 'lime')
axs[2].set_xlabel('PWM')
axs[2].set_ylabel('Kracht [mN]')
axs[2].set_title('Kleine Motor')
axs[2].grid(True)
axs[2].legend()


fig.suptitle('Vergelijking tussen de motoren [1.1A, 6V, standaard propellor en groene propellor]')
# Adjust layout
plt.tight_layout()
