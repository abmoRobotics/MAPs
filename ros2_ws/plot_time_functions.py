import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import math
import pandas as pd
# Define the functions as shown in the previous response

# Time remaining until deadline
def time_remaining_priority(task_deadline_seconds):
    return task_deadline_seconds

# Elapsed time since creation
def elapsed_time_priority(task_creation_seconds):
    return -task_creation_seconds

# Proportional urgency (time remaining divided by total time allotted)
def proportional_urgency_priority(task_creation_seconds, task_deadline_seconds):
    time_remaining = task_deadline_seconds - task_creation_seconds
    total_time_allotted = task_deadline_seconds
    return time_remaining / total_time_allotted

# Weighted combination of time remaining and elapsed time
def weighted_combination_priority(task_creation_seconds, task_deadline_seconds, weight=0.2):
    time_remaining = task_deadline_seconds - task_creation_seconds
    elapsed_time = task_creation_seconds
    return weight * time_remaining - (1 - weight) * elapsed_time

# Exponential decay of importance
def exponential_decay_priority(task_deadline_seconds, half_life_seconds=20):
    decay_rate = math.log(2) / half_life_seconds
    return math.exp(-decay_rate * task_deadline_seconds)

# Create a range of input seconds
input_seconds = np.arange(0, 200, 1)

# Calculate priority scores for each function
time_remaining_scores = [time_remaining_priority(x) for x in input_seconds]
elapsed_time_scores = [elapsed_time_priority(x) for x in input_seconds]
proportional_urgency_scores = [proportional_urgency_priority(x, 200) for x in input_seconds]
weighted_combination_scores = [weighted_combination_priority(x, 200) for x in input_seconds]
exponential_decay_scores = [exponential_decay_priority(x) for x in input_seconds]

# # Combine the input seconds and priority scores into a single dataset
# data = np.column_stack((input_seconds, time_remaining_scores, elapsed_time_scores, 
#                         proportional_urgency_scores, weighted_combination_scores, exponential_decay_scores))
# columns = ["Input Seconds", "Time Remaining", "Elapsed Time", "Proportional Urgency",
#            "Weighted Combination", "Exponential Decay"]
# data = pd.DataFrame(data, columns=columns)

# # Melt the dataset into a long format suitable for Seaborn line plots
# data_melted = data.melt(id_vars="Input Seconds", var_name="Function", value_name="Priority Score")

# # Plot the functions using Seaborn line plot
# plt.figure(figsize=(10, 6))
# sns.lineplot(data=data_melted, x="Input Seconds", y="Priority Score", hue="Function")
# plt.title("Temporal Priority Score Functions")
# plt.show()

# Create a dictionary to store the functions and their scores
function_dict = {"Time Remaining": time_remaining_scores,
                 "Elapsed Time": elapsed_time_scores,
                 "Proportional Urgency": proportional_urgency_scores,
                 "Weighted Combination": weighted_combination_scores,
                 "Exponential Decay": exponential_decay_scores}

# Create separate plots for each function
fig, axes = plt.subplots(nrows=3, ncols=2, figsize=(14, 16))
axes = axes.flatten()

sns.set_theme(style="darkgrid")
for idx, (function_name, scores) in enumerate(function_dict.items()):
    ax = axes[idx]
    sns.lineplot(x=input_seconds, y=scores, ax=ax)
    ax.set_title(function_name)
    ax.set_xlabel("Input Seconds")
    ax.set_ylabel("Priority Score")

# Remove the extra empty subplot
axes[-1].axis("off")

plt.tight_layout()
plt.show()