import os
import fonction_RPM as fonc_RPM
import matplotlib.pyplot as plt
import numpy as np
from tqdm import tqdm

def get_base_names(folder_path):
    if not os.path.exists(folder_path):
        print(f"Error: The directory '{folder_path}' does not exist.")
        return []

    unique_base_names = set()
    
    for file_name in os.listdir(folder_path):
        clean_name = file_name.lower().strip()
        if "_user" in clean_name and clean_name.endswith(".csv"):
            parts = file_name.split('_')
            if len(parts) >= 3:
                base_name = f"{parts[0]}_{parts[1]}_{parts[2]}_{parts[3]}"
                unique_base_names.add(base_name)
                
    final_list = list(unique_base_names)
    
    # SORTING LOGIC
    # Sort the list primarily by the user name (index 1), then by date (index 0)
    final_list.sort(key=lambda x: (x.split('_')[1], x.split('_')[0]))
    
    return final_list

# --- Initialization ---
target_folder = "./data/tripod"
experiment_list = get_base_names(target_folder)

print(f"Unique experiments found ({len(experiment_list)}):")

rpm_list = []
ref_rpm_list = []
user_list = [] # To keep track of the user for each experiment

# ==========================================
# 1. ANALYSIS AND CALCULATIONS
# ==========================================
for exp in tqdm(experiment_list, desc="Analyzing UWB signals", unit="exp"):
    
    # Store the current user (e.g., "userA")
    current_user = exp.split('_')[1]
    user_list.append(current_user)
    
    rpm, ref_rpm, breath_count, ref_breath_count = fonc_RPM.RPM(exp, fig=False)
    
    rpm_list.append(rpm)
    ref_rpm_list.append(ref_rpm)

# ==========================================
# 2. GLOBAL DATA PREPARATION
# ==========================================

users_to_ignore = ["userD", "userJ"] 

# Create new "clean" lists (excluding ignored users)
clean_rpm_list = []
clean_ref_rpm_list = []
clean_user_list = []
clean_experiment_list = []

# Iterate through raw results and retain only those not in the ignore list
for i in range(len(user_list)):
    if user_list[i] not in users_to_ignore:
        clean_rpm_list.append(rpm_list[i])
        clean_ref_rpm_list.append(ref_rpm_list[i])
        clean_user_list.append(user_list[i])
        clean_experiment_list.append(experiment_list[i])

print(f"{len(user_list) - len(clean_user_list)} experiments ignored.")
print(f"{len(clean_user_list)} experiments retained for global analysis.")

# Replace old variables with the cleaned arrays for subsequent calculations
radar_rpm = np.array(clean_rpm_list)
ref_rpm_arr = np.array(clean_ref_rpm_list)
user_list = clean_user_list # Essential for correct plotting
experiment_list = clean_experiment_list

# --- Error Calculation ---
percentage_errors = np.abs(radar_rpm - ref_rpm_arr) / ref_rpm_arr * 100
mean_error = np.mean(percentage_errors)

print(f"\nNew Global Mean Error (after filtering): {mean_error:.2f} %")

# --- Calculate formatting for user separation on the plot ---
separator_positions = []
user_labels = []
label_positions = []

current_user_group = user_list[0]
zone_start = 0

for i, user in enumerate(user_list):
    if user != current_user_group:
        separator_positions.append(i - 0.5) # The dividing line
        label_positions.append((zone_start + i - 1) / 2) # Center of the zone for the label
        user_labels.append(current_user_group)
        current_user_group = user
        zone_start = i
# Append logic for the final user:
label_positions.append((zone_start + len(user_list) - 1) / 2)
user_labels.append(current_user_group)

# ==========================================
# 3. DASHBOARD CREATION
# ==========================================
x = np.arange(len(experiment_list))
bar_width = 0.35

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 10), gridspec_kw={'height_ratios': [2, 1]})

# --- PLOT 1: RPM ---
ax1.bar(x - bar_width/2, radar_rpm, bar_width, label="UWB Radar", color='#1f77b4')
ax1.bar(x + bar_width/2, ref_rpm_arr, bar_width, label="Ground Truth", color='#ff7f0e')

# Draw vertical separation lines for different users
for pos in separator_positions:
    ax1.axvline(x=pos, color='grey', linestyle='-', linewidth=1.5, alpha=0.5)

# Display user names at the bottom of the X-axis (centered for each group)
ax1.set_xticks(label_positions)
ax1.set_xticklabels(user_labels, rotation=45, fontweight='bold')
ax1.set_ylabel('RPM')
ax1.set_title('Respiration Rate Comparison (Grouped by User)', fontsize=14, fontweight='bold')
ax1.legend()
ax1.grid(axis='y', linestyle='--', alpha=0.7)

# --- PLOT 2: ERRORS (%) ---
error_colors = ['green' if err < 10 else 'red' for err in percentage_errors]
ax2.bar(x, percentage_errors, color=error_colors, alpha=0.7)

for pos in separator_positions:
    ax2.axvline(x=pos, color='grey', linestyle='-', linewidth=1.5, alpha=0.5)

ax2.axhline(mean_error, color='black', linestyle='--', label=f'Global Mean ({mean_error:.1f}%)')
ax2.set_xticks(label_positions)
ax2.set_xticklabels(user_labels, rotation=45, fontweight='bold')
ax2.set_ylabel('Error (%)')
ax2.legend()
ax2.grid(axis='y', linestyle='--', alpha=0.7)

plt.tight_layout()
plt.show()