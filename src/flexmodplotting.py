# format from https://matplotlib.org/3.10.9/gallery/lines_bars_and_markers/barchart.html

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
import pandas as pd
from cmcrameri import cm
import scipy.stats as stats
from matplotlib.ticker import MaxNLocator

# load flexural modulus data from CSV file into a pandas DataFrame
df = pd.read_csv('data/flexModTable.csv')

all_dormant = df[df['Season'] == 'dormant']['Modulus'].tolist()
all_fruiting = df[df['Season'] == 'fruiting']['Modulus'].tolist()
dormant_mean = np.mean(all_dormant)
dormant_stdev = np.std(all_dormant)
fruiting_mean = np.mean(all_fruiting)
fruiting_stdev = np.std(all_fruiting)
print(f"Dormant mean: {dormant_mean:.2f} GPa, Dormant stdev: {dormant_stdev:.2f} GPa")

woody_dormant = df[(df['Age'] == 'woody') & (df['Season'] == 'dormant')]['Modulus'].tolist()
woody_fruiting = df[(df['Age'] == 'woody') & (df['Season'] == 'fruiting')]['Modulus'].tolist()
green_dormant = df[(df['Age'] == 'green') & (df['Season'] == 'dormant')]['Modulus'].tolist()
green_fruiting = df[(df['Age'] == 'green') & (df['Season'] == 'fruiting')]['Modulus'].tolist()
middle_dormant = df[(df['Age'] == 'middle') & (df['Season'] == 'dormant')]['Modulus'].tolist()
middle_fruiting = df[(df['Age'] == 'middle') & (df['Season'] == 'fruiting')]['Modulus'].tolist()

variety_colors = [cm.batlowKS.colors[0], cm.batlowKS.colors[1], cm.batlowKS.colors[2]]
age_colors = ['#887162', '#75a3c5', '#c5bd75'] 

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

SHOW_BOXPLOT = False

if SHOW_BOXPLOT:
    stat, p_value = stats.ttest_ind(all_dormant, all_fruiting, equal_var=False)
    print(f"T-test between dormant and fruiting seasons: stat={stat}, p-value={p_value}")

    # set the size of the figure and the layout to be constrained
    fig, ax = plt.subplots(figsize=(6, 2.25), layout='constrained')
    ax.boxplot([all_dormant, all_fruiting], 
            labels=['Dormant', 'Fruiting'], 
            vert=False, widths=[.525,.525])
    ax.set_xlabel('Flexural Modulus (GPa)')
    # plt.savefig(f"images/flexmodbyseasonv1_1.pdf", dpi=300)
    plt.show()

duke_dormant = df[(df['Season'] == 'dormant') & (df['Varietal'] == 'Duke')]['Modulus'].tolist()
# elliot_dormant = df[(df['Season'] == 'dormant') & (df['Varietal'] == 'Elliott')]['Modulus'].tolist()
# earliblue_dormant = df[(df['Season'] == 'dormant') & (df['Varietal'] == 'EarliBlue')]['Modulus'].tolist()
# legacy_dormant = df[(df['Season'] == 'dormant') & (df['Varietal'] == 'Legacy')]['Modulus'].tolist()

duke_fruiting = df[(df['Season'] == 'fruiting') & (df['Varietal'] == 'Duke')]['Modulus'].tolist()
liberty_fruiting = df[(df['Season'] == 'fruiting') & (df['Varietal'] == 'Liberty')]['Modulus'].tolist()
draper_fruiting = df[(df['Season'] == 'fruiting') & (df['Varietal'] == 'Draper')]['Modulus'].tolist()

stat, p_value = stats.ttest_ind(duke_fruiting, liberty_fruiting, equal_var=False)
print(f"T-test between Duke and Liberty fruiting: stat={stat}, p-value={p_value}")

stat, p_value = stats.ttest_ind(duke_fruiting, draper_fruiting, equal_var=False)
print(f"T-test between Duke and Draper fruiting: stat={stat}, p-value={p_value}")

stat, p_value = stats.ttest_ind(draper_fruiting, liberty_fruiting, equal_var=False)
print(f"T-test between Draper and Liberty fruiting: stat={stat}, p-value={p_value}")

# fig, axs = plt.subplots(3, 2, layout='constrained', sharex=True, figsize=(10, 12))

# vars = [woody_fruiting, duke_fruiting, middle_fruiting, liberty_fruiting, green_fruiting, draper_fruiting]
# names = ['Woody', 'Duke', 'Middle', 'Liberty', 'Green', 'Draper']

# for i, ax in enumerate(axs.flat):
#     ax.hist(vars[i], bins=8, alpha=0.7, color=age_colors[i%3] if i < 3 else variety_colors[i%3])
#     ax.axvline(np.mean(vars[i]), color='black', linestyle='dashed', linewidth=1, label=f'Mean: {np.mean(vars[i]):.2f} GPa')
#     ax.set_title(names[i], fontsize=14)
#     ax.yaxis.set_major_locator(MaxNLocator(integer=True))
#     if i == 0 or i == 2 or i == 4:
#         ax.set_ylabel('Sample Count')
#     ax.legend()

# axs[2, 0].set_xlabel('Flexural Modulus (GPa)')
# axs[2, 1].set_xlabel('Flexural Modulus (GPa)')

SHOW_EVENTPLOT = True

if SHOW_EVENTPLOT:
    data = [[woody_fruiting, middle_fruiting, green_fruiting], [duke_fruiting, liberty_fruiting, draper_fruiting]]
    labels = [['Woody', 'Middle', 'Green'], ['Duke', 'Liberty', 'Draper']]
    colors = [age_colors, variety_colors]

    fig2, axs2 = plt.subplots(1, 2, layout='constrained', sharex=True, figsize=(10, 5))

    for i, ax in enumerate(axs2):
        ec = ax.eventplot(data[i], colors=colors[i], orientation='horizontal', lineoffsets=[1, 2, 3], linelengths=0.8, linewidths=2)
        ax.eventplot([[np.mean(d)] for d in data[i]], colors='black', orientation='horizontal', lineoffsets=[1, 2, 3], linelengths=0.8, linewidths=2, linestyles='dashed')
        mean_handle = Line2D([0], [0], color='black', linewidth=2, linestyle='dashed')
        ax.legend(handles=ec + [mean_handle], labels=labels[i] + ['Average'], title='Age' if i == 0 else 'Variety', fontsize=12, loc='upper right', title_fontsize=12)
        ax.set_xlabel('Flexural Modulus (GPa)', fontsize=16)
        ax.yaxis.set_major_locator(plt.NullLocator())
        ax.tick_params(axis='x', labelsize=12)

    bfr = 0.2
    # x_max_plt1 = max(max(woody_fruiting), max(middle_fruiting), max(green_fruiting))
    x2 = max(max(duke_fruiting), max(liberty_fruiting), max(draper_fruiting))
    x1 = max(max(draper_fruiting), max(liberty_fruiting))
    axs2[1].plot([x1+bfr, x1+bfr*2, x1+bfr*2, x1+bfr], [2, 2, 3, 3], lw=1.5, color='black')
    axs2[1].plot([x2+bfr, x2+bfr*2, x2+bfr*2, x2+bfr], [1, 1, 2, 2], lw=1.5, color='black')
    axs2[1].text(x2+0.5, 1.5, '*', ha='left', va='center', fontsize=16)
    axs2[1].text(x1+0.5, 2.5, '**', ha='left', va='center', fontsize=16)
    axs2[1].set_xlim(1, 6.2)
    axs2[1].text(0,0, 'Note: * p<0.008, ** p<0.004', fontsize=16, ha='left', va='bottom', transform=axs2[1].transAxes)
    plt.savefig(f"images/flexmodbyvarietyandage2_1.pdf", dpi=300)
    plt.show()

stat, p_value = stats.ttest_ind(duke_dormant, duke_fruiting, equal_var=True)
print(f"T-test between Duke dormant and fruiting: stat={stat}, p-value={p_value}")
print(f"Duke dormant mean: {np.mean(duke_dormant):.2f} GPa, Duke fruiting mean: {np.mean(duke_fruiting):.2f} GPa")


# fig, (ax1, ax2) = plt.subplots(1, 2, layout='constrained', sharey=True, sharex=True)
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


plt.show()




