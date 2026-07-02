import matplotlib.pyplot as plt
import numpy as np

woody_dormant = [4.52, 5.40, 5.63, 3.15, 7.31, 3.07]
woody_harvest = [1.68, 2.28, 3.07, 2.41, 2.56, 2.74, 4.04]
green_dormant = [5.40, 5.04, 5.36, 3.71, 6.08, 4.85]
green_harvest = [4.24, 4.47, 4.81, 5.17, 5.53]

season = ("Dormant (4 NHB varietals)", "Harvest (Duke)")
age = ("woody", "green")
age_means = {
    'woody': [np.mean(woody_dormant), np.mean(woody_harvest)],
    'green': [np.mean(green_dormant), np.mean(green_harvest)]
}
age_stdevs = { 
    'woody': [np.std(woody_dormant), np.std(woody_harvest)],
    'green': [np.std(green_dormant), np.std(green_harvest)]
}
season_means = {
    'dormant': [np.mean(woody_dormant), np.mean(green_dormant)],
    'harvest': [np.mean(woody_harvest), np.mean(green_harvest)]
}
season_stdevs = {
    'dormant': [np.std(woody_dormant), np.std(green_dormant)],
    'harvest': [np.std(woody_harvest), np.std(green_harvest)]
}

x_locs = np.arange(len(season))
bar_width = 0.35
multiplier = 0
colors = ['#887162', '#c5bd75']  # woody, green

fig, ax = plt.subplots(layout='constrained')

for (attribute, measurement), color in zip(age_means.items(), colors):
    offset = bar_width * multiplier
    rects = ax.bar(x_locs + offset, measurement, bar_width, label=attribute, color=color)
    ax.errorbar(x_locs + offset, measurement, yerr=age_stdevs[attribute], fmt='none', ecolor='black', capsize=5)
    ax.bar_label(rects, padding=-15, fmt='%.2f', label_type='center')
    multiplier += 1
# res = ax.grouped_bar(season_means, tick_labels=age, colors=['#5c4c3c', '#c5bd75'], group_spacing=0.5)

ax.set_ylabel('Mean Flexural Modulus (GPa)')
ax.set_title('Mean Flexural Modulus by Age and Season')
ax.set_xticks(x_locs + bar_width/2, labels=season)
ax.set_xticklabels(season)
ax.legend(title='Age')
plt.show()

