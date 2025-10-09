# filePlotter.py (add/replace with this)

import matplotlib.pyplot as plt
from utilities import FileReader
import argparse
import os

def _time_from_last_col(values):
    first_stamp = values[0][-1]
    # Convert ns → s if they look huge; else keep as-is
    scale = 1e9 if float(first_stamp) > 1e12 else 1.0
    return [ (row[-1] - first_stamp) / scale for row in values ], ("Time (s)" if scale==1e9 else "Time (ns)")

def plot_one(ax, filename):
    headers, values = FileReader(filename).read_file()
    if not headers or not values:
        ax.set_title(f"No data: {os.path.basename(filename)}")
        ax.axis("off")
        return

    t, xlabel = _time_from_last_col(values)

    for i in range(0, len(headers) - 1):
        name = headers[i]
        # quick unit tagging based on column name
        if "x" in name.lower() or "y" in name.lower():
            unit = "m/s^2"
        else:
            unit = "rad/s"
        y = [row[i] for row in values]
        ax.plot(t, y, label=f"{name} ({unit})", linewidth=1.3)

    ax.set_title(f"Logged Sensor Data – {os.path.basename(filename)}")
    ax.set_xlabel(xlabel)
    ax.set_ylabel("Measured Value")
    ax.grid(True, linestyle=":", linewidth=0.8)
    ax.legend(loc="best", frameon=True, fontsize=9)

def plot_three(f1, f2, f3):
    fig, axes = plt.subplots(1, 3, figsize=(18, 5), constrained_layout=True)
    files = [f1, f2, f3]
    for ax, fn in zip(axes, files):
        plot_one(ax, fn)
    plt.show()

def plot_errors(filename):
    # keep your original single-plot behavior
    fig, ax = plt.subplots(figsize=(10,6))
    plot_one(ax, filename)
    plt.show()

if __name__=="__main__":
    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True,
                        help='List of files to process (pass 3 files to show them together)')
    args = parser.parse_args()

    print("plotting the files", args.files)

    if len(args.files) == 3:
        plot_three(args.files[0], args.files[1], args.files[2])
    else:
        # fallback: plot each in its own window
        for filename in args.files:
            plot_errors(filename)
