import scipy.io
import matplotlib.pyplot as plt
import numpy as np
import os

# Define the directory and label
current_dir = os.path.dirname(os.path.realpath(__file__)) #if this code is in the same directory as your main.py file.
print(current_dir)
model_dir = os.path.join(current_dir, "model/marl_model/")

# Load data from .mat files
reward_data = scipy.io.loadmat(os.path.join(model_dir, "reward.mat"))['reward'].flatten()

# Plotting examples (adjust as needed)

# 1. Plot the reward per episode
plt.figure(figsize=(10, 5))
plt.plot(reward_data)
plt.title("Reward per Episode (Testing)")
plt.xlabel("Episode")
plt.ylabel("Reward")
plt.grid(True)
plt.show()

# 2. Plot AoI for each platoon over episodes
aoi_data = scipy.io.loadmat(os.path.join(model_dir, "AoI.mat"))['AoI']
plt.figure(figsize=(10, 5))
for i in range(aoi_data.shape[0]):
    plt.plot(aoi_data[i, :], label=f"Platoon {i+1}")
plt.title("AoI per Episode (Testing)")
plt.xlabel("Episode")
plt.ylabel("AoI")
plt.legend()
plt.grid(True)
plt.show()
