
import csv
from pathlib import Path
import matplotlib.pyplot as plt

# Find the CSV file in the same folder
folder = Path(__file__).resolve().parent
csv_path = folder / "speed_log.csv"

# Read simulation data
times = []
targets = []
speeds = []

with open(csv_path, "r") as file:
    reader = csv.DictReader(file)

    for row in reader:
        times.append(float(row["Time (s)"]))
        targets.append(float(row["Target Speed (mm/s)"]))
        speeds.append(float(row["Actual Speed (mm/s)"]))

# Smooth the readings with a 5-sample moving average
smoothed = []

for i in range(len(speeds)):
    recent = speeds[max(0, i - 4):i + 1]
    smoothed.append(sum(recent) / len(recent))

# Create the graph
plt.figure(figsize=(10, 5))

plt.plot(times, targets, "--", label="Target Speed")
plt.plot(times, speeds, alpha=0.5, label="Encoder Speed")
plt.plot(times, smoothed, label="Smoothed Speed")

# Show the required settling-time threshold
plt.axvline(0.5, linestyle=":", label="0.5 s Threshold")

plt.title("Wizard Chess - PID Motor Speed Simulation")
plt.xlabel("Time (seconds)")
plt.ylabel("Wheel Speed (mm/s)")
plt.legend()
plt.grid(True)
plt.tight_layout()

# Save the graph
output = folder / "speed_plot.png"
plt.savefig(output, dpi=150)
plt.close()

print("Graph saved to:", output)
print("Plotting complete!")


# Analyze PID settling time
print("\n--- PID Performance Analysis ---")

tolerance = 0.05  # 5% speed error
settling_time = None

# Find when the smoothed speed enters the 5% range
# and remains there until the simulation ends.
for i in range(len(times)):
    stays_in_range = all(
        abs(smoothed[j] - targets[j])
        <= tolerance * abs(targets[j])
        for j in range(i, len(times))
    )

    if stays_in_range:
        settling_time = times[i]
        break

if settling_time is None:
    print("Settling time: Not reached")
    print("Settling test: FAIL")
else:
    print("Settling time:",
          round(settling_time, 3), "seconds")

    if settling_time <= 0.5:
        print("Settling test: PASS")
    else:
        print("Settling test: FAIL")

# Check speed variation over the final 0.5 seconds
final_speeds = [
    smoothed[i]
    for i in range(len(times))
    if times[i] >= times[-1] - 0.5
]

variation = max(final_speeds) - min(final_speeds)

print("Final smoothed speed variation:",
      round(variation, 2), "mm/s")

print("Analysis complete!")
