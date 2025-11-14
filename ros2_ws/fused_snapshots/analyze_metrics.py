import pandas as pd
import matplotlib.pyplot as plt

# Load CSV file
df = pd.read_csv('validation_metrics.csv')

# Normalize column names: lowercase + remove spaces
df.columns = df.columns.str.strip().str.lower().str.replace(' ', '_')

# Print detected columns to verify
print("Detected columns:", df.columns.tolist())

# Extract frame index
frames = df['frame']

# Plot Chamfer Distance
plt.figure(figsize=(8, 4))
plt.plot(frames, df['chamfer_distance'], label='Chamfer Distance', linewidth=2)
plt.xlabel('Frame')
plt.ylabel('Chamfer Distance')
plt.title('Reconstruction Accuracy Over Time')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig('chamfer_distance_plot.png', dpi=300)
plt.show()

# Plot Overlap Ratio
plt.figure(figsize=(8, 4))
plt.plot(frames, df['overlap_ratio'], color='green', label='Overlap Ratio', linewidth=2)
plt.xlabel('Frame')
plt.ylabel('Overlap Ratio')
plt.title('Temporal Consistency Over Time')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig('overlap_ratio_plot.png', dpi=300)
plt.show()

# Plot Delta Depth
plt.figure(figsize=(8, 4))
plt.plot(frames, df['delta_depth'], color='orange', label='Delta Depth', linewidth=2)
plt.xlabel('Frame')
plt.ylabel('Delta Depth')
plt.title('Depth Stability Over Time')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig('delta_depth_plot.png', dpi=300)
plt.show()

# Plot Convergence Score
plt.figure(figsize=(8, 4))
plt.plot(frames, df['convergence_score'], color='purple', label='Convergence Score', linewidth=2)
plt.xlabel('Frame')
plt.ylabel('Convergence Score')
plt.title('Temporal Convergence')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig('convergence_score_plot.png', dpi=300)
plt.show()
