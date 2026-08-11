# format from https://matplotlib.org/3.10.9/gallery/lines_bars_and_markers/barchart.html

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from cmcrameri import cm
import scipy.stats as stats

# load flexural modulus data from CSV file into a pandas DataFrame
df = pd.read_csv('data/flexModTable.csv')

all_dormant = df[df['Season'] == 'dormant']['Modulus'].tolist()
all_fruiting = df[df['Season'] == 'fruiting']['Modulus'].tolist()
dormant_mean = np.mean(all_dormant)
dormant_stdev = np.std(all_dormant)
fruiting_mean = np.mean(all_fruiting)
fruiting_stdev = np.std(all_fruiting)

woody_dormant = df[(df['Age'] == 'woody') & (df['Season'] == 'dormant')]['Modulus'].tolist()
woody_fruiting = df[(df['Age'] == 'woody') & (df['Season'] == 'fruiting')]['Modulus'].tolist()
green_dormant = df[(df['Age'] == 'green') & (df['Season'] == 'dormant')]['Modulus'].tolist()
green_fruiting = df[(df['Age'] == 'green') & (df['Season'] == 'fruiting')]['Modulus'].tolist()
middle_dormant = df[(df['Age'] == 'middle') & (df['Season'] == 'dormant')]['Modulus'].tolist()
middle_fruiting = df[(df['Age'] == 'middle') & (df['Season'] == 'fruiting')]['Modulus'].tolist()

# season = ("Dormant (Legacy, Duke, EarliBlue, Elliott)", "Fruiting (Duke, Liberty, Draper)")
season = ("Dormant (Duke)", "Fruiting (Duke)")
age = ("woody", "green", "middle")
age_means = {
    'woody': [np.mean(woody_dormant), np.mean(woody_fruiting)],
    'green': [np.mean(green_dormant), np.mean(green_fruiting)],
    'middle': [np.mean(middle_dormant), np.mean(middle_fruiting)]
}
age_stdevs = { 
    'woody': [np.std(woody_dormant), np.std(woody_fruiting)],
    'green': [np.std(green_dormant), np.std(green_fruiting)],
    'middle': [np.std(middle_dormant), np.std(middle_fruiting)]
}
season_means = {
    'dormant': [np.mean(woody_dormant), np.mean(green_dormant)],
    'fruiting': [np.mean(woody_fruiting), np.mean(green_fruiting)]
}
season_stdevs = {
    'dormant': [np.std(woody_dormant), np.std(green_dormant)],
    'fruiting': [np.std(woody_fruiting), np.std(green_fruiting)]
}

stat, p_value = stats.ttest_ind(all_dormant, all_fruiting, equal_var=False)
print(f"T-test between dormant and fruiting seasons: stat={stat}, p-value={p_value}")

# duke_dormant = df[(df['Season'] == 'dormant') & (df['Varietal'] == 'Duke')]['Modulus'].tolist()
# elliot_dormant = df[(df['Season'] == 'dormant') & (df['Varietal'] == 'Elliott')]['Modulus'].tolist()
# earliblue_dormant = df[(df['Season'] == 'dormant') & (df['Varietal'] == 'EarliBlue')]['Modulus'].tolist()
# legacy_dormant = df[(df['Season'] == 'dormant') & (df['Varietal'] == 'Legacy')]['Modulus'].tolist()

# duke_fruiting = df[(df['Season'] == 'fruiting') & (df['Varietal'] == 'Duke')]['Modulus'].tolist()
# liberty_fruiting = df[(df['Season'] == 'fruiting') & (df['Varietal'] == 'Liberty')]['Modulus'].tolist()
# draper_fruiting = df[(df['Season'] == 'fruiting') & (df['Varietal'] == 'Draper')]['Modulus'].tolist()

# print(f"Duke Fruiting Mean and Stdev: {np.mean(duke_fruiting)}, {np.std(duke_fruiting)}")
# print(f"Liberty Fruiting Mean and Stdev: {np.mean(liberty_fruiting)}, {np.std(liberty_fruiting)}")
# print(f"Draper Fruiting Mean and Stdev: {np.mean(draper_fruiting)}, {np.std(draper_fruiting)}")

# fig, (ax, ax2) = plt.subplots(1, 2, layout='constrained', sharey=True, sharex=True)
# facecolors = [cm.batlowKS.colors[0], cm.batlowKS.colors[1], cm.batlowKS.colors[2], cm.batlowKS.colors[3]]
# ax.hist([duke_dormant, elliot_dormant, earliblue_dormant, legacy_dormant], 
#         bins=10, 
#         histtype='barstacked', 
#         alpha=0.7, 
#         label=['Duke', 'Elliot', 'EarliBlue', 'Legacy'], 
#         color=facecolors, 
#         hatch = ['|', '.', 'x', '*'])
# ax.legend(title='Variety')
# ax.set_xlabel('Flexural Modulus (GPa)')
# ax.set_ylabel('Count')
# ax.set_title('Dormant Season')

# # Right plot — configure as needed
# ax2.hist([duke_fruiting, liberty_fruiting, draper_fruiting], 
#          bins=10, 
#          histtype='barstacked', 
#          alpha=0.7, 
#          label=['Duke', 'Liberty', 'Draper'], 
#          color=[facecolors[0], cm.batlowKS.colors[4], cm.batlowKS.colors[5]],
#          hatch=['|', 'O', '/'])
# ax2.set_xlabel('Flexural Modulus (GPa)')
# ax2.set_ylabel('Count')
# ax2.legend(title='Variety')
# ax2.set_title('Fruiting Season')

# # plt.show()

# fig, (ax3, ax4) = plt.subplots(1, 2, layout='constrained', sharey=True, sharex=True)
# age_colors = ['#887162', '#75a3c5', '#c5bd75'] 
# ax3.hist([woody_dormant, middle_dormant, green_dormant], 
#          bins=10, 
#          histtype='barstacked', 
#          alpha=0.7, 
#          label=['Woody', 'Middle', 'Green'], 
#          color=age_colors)
# ax3.set_xlabel('Flexural Modulus (GPa)')
# ax3.axvline(dormant_mean, color='black', linestyle='dashed', linewidth=1, label=f'Mean: {dormant_mean:.2f} GPa')
# ax3.set_ylabel('Count')
# ax3.legend(title='Age')
# ax3.set_title('Dormant Season')

# ax4.hist([woody_fruiting, middle_fruiting, green_fruiting], 
#          bins=10, 
#          histtype='barstacked', 
#          alpha=0.7, 
#          label=['Woody', 'Middle', 'Green'], 
#          color=age_colors)
# ax4.set_xlabel('Flexural Modulus (GPa)')
# ax4.axvline(fruiting_mean, color='black', linestyle='dashed', linewidth=1, label=f'Mean: {fruiting_mean:.2f} GPa')
# ax4.set_ylabel('Count')
# ax4.legend(title='Age')
# ax4.set_title('Fruiting Season')

# # fruiting_woody_df = df[(df['Season'] == 'fruiting') & (df['Age'] == 'woody')]

# # fig, ax = plt.subplots(layout='constrained')
# # for i, varietal in enumerate(fruiting_woody_df['Varietal'].unique()):
# #     subset = fruiting_woody_df[fruiting_woody_df['Varietal'] == varietal]
# #     ax.scatter(subset['Diameter'], subset['Modulus'], color=cm.batlowKS.colors[i], alpha=0.7, label=varietal)
# # ax.legend(title='Varietal')
# # ax.set_xlabel('Diameter (mm)')
# # ax.set_ylabel('Flexural Modulus (GPa)')
# # ax.set_title('Fruiting Season Woody Age Flexural Modulus vs Diameter')  


# plt.show()




