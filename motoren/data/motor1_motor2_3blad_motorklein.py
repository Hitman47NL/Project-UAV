import numpy as np
import matplotlib.pyplot as plt

# Import and use the code from the 'data' folder
import data.kleine_motor as kleine_motor
import data.motor1_3blad as motor1_3blad
import data.motor2_3blad as motor2_3blad

# Create subplots
fig, axs = plt.subplots(1, 3, figsize=(16, 6))

# Plot data for motor 1
axs[0].plot(motor1_3blad.meting_1_3blad_pos, motor1_3blad.PWM_1_3blad_pos, marker= 'o', label='Motor 1 [+]', color = 'magenta')
axs[0].plot(motor1_3blad.meting_1_3blad_neg,motor1_3blad.PWM_1_3blad_neg, marker= 'o', label='Motor 1 [-]', color = 'lime')
axs[0].set_xlabel('PWM')
axs[0].set_ylabel('Kracht [mN]')
axs[0].set_title('Motor 1, 3 blade propellor')
axs[0].grid(True)
axs[0].legend()

# Plot data for motor 2
axs[1].plot(motor2_3blad.PWM_2_3blad_pos, motor2_3blad.meting_2_3blad_pos, marker= 'o', label='Motor 2 [+]', color = 'magenta')
axs[1].plot(motor2_3blad.PWM_2_3blad_neg, motor2_3blad.meting_2_3blad_neg, marker= 'o', label='Motor 2 [-]', color = 'lime')
axs[1].set_xlabel('PWM')
axs[1].set_ylabel('Kracht [mN]')
axs[1].set_title('Motor 2, 3 blade propellor')
axs[1].grid(True)
axs[1].legend()

# Plot data for kleine motor
axs[2].plot(kleine_motor.PWM_klein_pos, kleine_motor.meting_klein_pos, marker= 'o', label='Kleine Motor [+]', color = 'magenta')
axs[2].plot(kleine_motor.PWM_klein_neg, kleine_motor.meting_klein_neg, marker= 'o', label='Kleine Motor [-]', color = 'lime')
axs[2].set_xlabel('PWM')
axs[2].set_ylabel('Kracht [mN]')
axs[2].set_title('Kleine Motor')
axs[2].grid(True)
axs[2].legend()

fig.suptitle('Vergelijking tussen de motoren [1.1A, 6V, 3 blade propellor]')
# Adjust layout
plt.tight_layout()
