#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import numpy as np

# ───────────────────────────────
# Setup
# ───────────────────────────────
DATA_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = DATA_DIR / "analysis_results"
OUTPUT_DIR.mkdir(exist_ok=True)

FILES = {
    "MASt3R": DATA_DIR / "mast3r_metrics.csv",
    "Temporal Fusion (Fixed)": DATA_DIR / "fixed_metrics.csv",
    "Temporal Fusion (Adaptive)": DATA_DIR / "validation_metrics.csv",
}

# ───────────────────────────────
# Load and normalize
# ───────────────────────────────
def load_metrics(label, filepath):
    df = pd.read_csv(filepath)
    df.columns = [c.strip().lower() for c in df.columns]  # normalize headers
    rename_map = {
        "chamferdistance": "chamfer_distance",
        "overlapratio": "overlap_ratio",
        "deltadepth": "delta_depth",
        "convergencescore": "convergence_score",
    }
    df.rename(columns=rename_map, inplace=True)
    df["method"] = label
    return df

dfs = []
for name, path in FILES.items():
    if path.exists():
        print(f"✅ Loaded {path.name}")
        dfs.append(load_metrics(name, path))
    else:
        print(f"⚠️ Missing file: {path}")

data = pd.concat(dfs, ignore_index=True)
print(f"Combined dataset shape: {data.shape}")

# ───────────────────────────────
# Compute summary
# ───────────────────────────────
summary = data.groupby("method").agg(["mean", "std"]).reset_index()
summary.columns = ['_'.join(col).strip() if col[1] else col[0] for col in summary.columns.values]
summary_path = OUTPUT_DIR / "metrics_summary.csv"
summary.to_csv(summary_path, index=False)
print(f"Saved summary to {summary_path}")

# ───────────────────────────────
# Plot setup
# ───────────────────────────────
sns.set(style="whitegrid", context="talk")
pdf_path = OUTPUT_DIR / "metrics_comparison_plots.pdf"
from matplotlib.backends.backend_pdf import PdfPages

metrics_to_plot = [
    ("chamfer_distance", "Chamfer Distance ↓"),
    ("delta_depth", "Δ Depth ↓"),
    ("overlap_ratio", "Overlap Ratio ↑"),
    ("convergence_score", "Convergence Score ↑"),
]

# ───────────────────────────────
# Plot all comparisons
# ───────────────────────────────
with PdfPages(pdf_path) as pdf:
    for metric, label in metrics_to_plot:
        if metric not in data.columns:
            print(f"Skipping missing metric: {metric}")
            continue

        # Boxplot
        plt.figure(figsize=(10, 6))
        sns.boxplot(
            x="method", y=metric, data=data, palette="Set2", showmeans=True,
            meanprops={"marker": "o", "markerfacecolor": "black", "markersize": 8}
        )
        plt.title(f"{label} Comparison", fontsize=18, weight='bold')
        plt.ylabel(label)
        plt.xlabel("")
        plt.tight_layout()
        pdf.savefig()
        plt.close()

        # Bar chart (mean values)
        plt.figure(figsize=(8, 5))
        mean_values = data.groupby("method")[metric].mean().sort_values()
        bar_colors = sns.color_palette("Set2", n_colors=len(mean_values))
        mean_values.plot(kind="bar", color=bar_colors)
        plt.title(f"Average {label}", fontsize=16, weight='bold')
        plt.ylabel(label)
        plt.xticks(rotation=20, ha='right')
        plt.tight_layout()
        pdf.savefig()
        plt.close()

    # ───────────────────────────────
    # Ranking Table
    # ───────────────────────────────
    rank_data = []
    for metric, label in metrics_to_plot:
        if metric not in data.columns:
            continue
        mean_vals = data.groupby("method")[metric].mean()
        ascending = True if "distance" in metric or "delta" in metric else False
        ranking = mean_vals.rank(ascending=ascending).astype(int)
        rank_data.append(pd.DataFrame({
            "Metric": label,
            "Method": mean_vals.index,
            "Mean": mean_vals.values,
            "Rank": ranking.values
        }))

    if rank_data:
        rank_table = pd.concat(rank_data)
        rank_table = rank_table.sort_values(["Metric", "Rank"])
        rank_path = OUTPUT_DIR / "ranking_table.csv"
        rank_table.to_csv(rank_path, index=False)
        print(f"🏆 Ranking table saved to {rank_path}")

        # Plot ranking table heatmap
        pivot_rank = rank_table.pivot(index="Metric", columns="Method", values="Rank")
        plt.figure(figsize=(8, 5))
        sns.heatmap(pivot_rank, annot=True, cmap="YlGnBu", cbar=False)
        plt.title("Ranking Table (Lower = Better)", fontsize=16, weight='bold')
        pdf.savefig()
        plt.close()

    # ───────────────────────────────
    # Correlation Heatmap
    # ───────────────────────────────
    numeric_data = data[[m for m, _ in metrics_to_plot if m in data.columns]]
    if not numeric_data.empty:
        corr = numeric_data.corr()
        plt.figure(figsize=(6, 5))
        sns.heatmap(corr, annot=True, cmap="coolwarm", fmt=".2f")
        plt.title("Metric Correlation Heatmap", fontsize=16, weight='bold')
        plt.tight_layout()
        pdf.savefig()
        plt.close()

print(f"\nAll plots saved to {pdf_path}")
print(f"Complete quantitative analysis ready for your thesis!")
