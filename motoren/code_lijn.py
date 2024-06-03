import numpy as np
from scipy.stats import linregress
import matplotlib.pyplot as plt

# Updated data
PWM_updated = np.array([51, 102, 153, 204, 255])
Force_updated = np.array([4.54, 83.92, 129.29, 147.43, 159.78])

# Perform linear regression on the updated data
slope_updated, intercept_updated, r_value_updated, p_value_updated, std_err_updated = linregress(PWM_updated, Force_updated)

# Calculate fitted values
fit_values_updated = slope_updated * PWM_updated + intercept_updated

# Plotting the updated data points and the linear fit
plt.figure(figsize=(10, 6))
plt.scatter(PWM_updated, Force_updated, color='blue', label='Updated Data Points')
plt.plot(PWM_updated, fit_values_updated, color='red', label=f'Linear Fit: F = {slope_updated:.4f}*PWM + {intercept_updated:.4f}')
plt.xlabel('PWM')
plt.ylabel('Force (N)')
plt.title('Linear Fit of Updated PWM vs Force')
plt.legend()
plt.grid(True)
plt.show()

(slope_updated, intercept_updated)
