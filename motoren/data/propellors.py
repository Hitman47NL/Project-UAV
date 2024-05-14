import matplotlib.pyplot as plt

# Data neg
categories1 = ['standaard', '2-bladed', '3-bladed', '6-bladed']
values1 = [165.58, 215.48, 244.97, 267.65 ]

# Data pos
categories2 = ['standaard', '2-bladed', '3-bladed', '6-bladed']
values2 = [102.07, 185.99, 204.14, 238.16]

# Create subplots
fig, axs = plt.subplots(1, 2, figsize=(8, 10))  # 2 rows, 1 column

# Plot first data in the first subplot
bars1 = axs[0].bar(categories1, values1, color='skyblue')
axs[0].set_title('Max kracht propellors [-]')
axs[0].set_xlabel('Propellors')
axs[0].set_ylabel('Kracht [N]')

# Annotate each bar with its value in the first subplot
for bar in bars1:
    height = bar.get_height()
    axs[0].text(bar.get_x() + bar.get_width()/2, height, '%.2f' % height,
                ha='center', va='bottom')

# Plot second data in the second subplot
bars2 = axs[1].bar(categories2, values2, color='salmon')
axs[1].set_title('Max kracht propellors [+]')
axs[1].set_xlabel('Propellors')
axs[1].set_ylabel('Kracht [N]')

# Annotate each bar with its value in the second subplot
for bar in bars2:
    height = bar.get_height()
    axs[1].text(bar.get_x() + bar.get_width()/2, height, '%.2f' % height,
                ha='center', va='bottom')

# Adjust layout
plt.tight_layout()

