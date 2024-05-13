import matplotlib.pyplot as plt

# Data
categories = ['standaard', '2-bladed', '3-bladed', '6-bladed']
values = [117.95, 233.63, 278.99, 229.09]

# Create bar chart
plt.figure(figsize=(8, 6))
plt.bar(categories, values, color='skyblue')

# Add labels and title
plt.xlabel('Propellors')
plt.ylabel('Kracht [N]')
plt.title('Propellor vs propellor [+]')

for bar in plt.bar(categories, values):
    height = bar.get_height()
    plt.text(bar.get_x() + bar.get_width()/2, height, '{:.2f}'.format(height),
             ha='center', va='bottom')
