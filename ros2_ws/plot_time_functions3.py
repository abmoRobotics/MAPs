import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import math
import pandas as pd

# Define the additional functions as shown in the previous response

def linear_decay_priority(task_deadline_seconds, total_time_seconds):
    time_remaining = task_deadline_seconds
    return total_time_seconds - time_remaining


def logistic_decay_priority(task_deadline_seconds, midpoint_seconds, steepness):
    time_remaining = task_deadline_seconds
    return -1 / (1 + math.exp(-steepness * (time_remaining - midpoint_seconds))) + 1


def time_remaining_squared_priority(task_deadline_seconds):
    time_remaining = task_deadline_seconds
    return time_remaining ** 2

def harmonic_priority(task_creation_seconds, task_deadline_seconds, alpha=1):
    time_remaining = task_deadline_seconds - task_creation_seconds
    return (time_remaining + alpha) / (task_deadline_seconds + alpha)


def inverse_time_remaining_priority(task_deadline_seconds, epsilon=1e-9):
    time_remaining = task_deadline_seconds
    return 1 / (time_remaining + epsilon)

# Create a range of input seconds
input_seconds = np.arange(0, 200, 1)

# Calculate priority scores for each function
linear_decay_scores = [linear_decay_priority(x, 200) for x in input_seconds]
logistic_decay_scores = [logistic_decay_priority(x, 100, 0.06) for x in input_seconds]
time_remaining_squared_scores = [time_remaining_squared_priority(x) for x in input_seconds]
harmonic_priority_scores = [harmonic_priority(36, x) for x in input_seconds]
inverse_time_remaining_scores = [inverse_time_remaining_priority(x) for x in input_seconds]

# # Create a dictionary to store the functions and their scores
# function_dict = {"Linear Decay": linear_decay_scores,
#                  "Logistic Decay": logistic_decay_scores,
#                  "Time Remaining Squared": time_remaining_squared_scores,
#                  "Harmonic Priority": harmonic_priority_scores,
#                  "Inverse Time Remaining": inverse_time_remaining_scores}

# # Create separate plots for each function
# fig, axes = plt.subplots(nrows=3, ncols=2, figsize=(14, 16))
# axes = axes.flatten()

# for idx, (function_name, scores) in enumerate(function_dict.items()):
#     ax = axes[idx]
#     sns.lineplot(x=input_seconds, y=scores, ax=ax)
#     ax.set_title(function_name)
#     ax.set_xlabel("Input Seconds")
#     ax.set_ylabel("Priority Score")

# # Remove the extra empty subplot
# axes[-1].axis("off")

# plt.tight_layout()
# plt.show()



# Combine the input seconds and priority scores into a single dataset
data = np.column_stack((input_seconds, linear_decay_scores, logistic_decay_scores,
                        time_remaining_squared_scores, harmonic_priority_scores, inverse_time_remaining_scores))
columns = ["Input Seconds", "Linear Decay", "Logistic Decay", "Time Remaining Squared",
           "Harmonic Priority", "Inverse Time Remaining"]
data = pd.DataFrame(data, columns=columns)

# Melt the dataset into a long format suitable for Seaborn line plots
data_melted = data.melt(id_vars="Input Seconds", var_name="Function", value_name="Priority Score")

# Plot the functions using Seaborn's relplot
g = sns.relplot(data=data_melted, x="Input Seconds", y="Priority Score", col="Function", kind="line", col_wrap=2, facet_kws={'ylim': (-0.1, 1.1)})
g.fig.subplots_adjust(top=0.9)  # Adjust the top of the subplots to create space for the title
g.fig.suptitle("Additional Temporal Priority Score Functions")
plt.show()