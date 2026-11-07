import csv
from pathlib import Path
from encoder import WheelEncoder
from pid_controller import PIDController

# Create simulated encoder and PID controller
encoder = WheelEncoder()
pid = PIDController(kp=0.005, ki=0.04)

TARGET_SPEED = 100.0
MAX_MOTOR_SPEED = 350.0
DT = 0.02  # 20 ms = 50 updates per second

actual_speed = 0.0
measured_speed = 0.0
leftover_counts = 0.0
speed_history = []
time_history = []
power_history = []

# Quadrature encoder states
states = [(0, 0), (0, 1), (1, 1), (1, 0)]
position = 0
encoder.update(*states[position])

print("Starting PID motor simulation...")
print("Time | Target | Measured | Power")

# Simulate 3 seconds
for step in range(150):
    # PID decides how much motor power to use
    power = pid.update(
        TARGET_SPEED, measured_speed, DT
    )

    # Simulate motor response with some inertia
    desired_speed = power * MAX_MOTOR_SPEED
    actual_speed += (
        (desired_speed - actual_speed) * DT / 0.15
    )

    # Convert simulated movement into encoder counts
    counts = (
        actual_speed * DT / encoder.mm_per_count
        + leftover_counts
    )

    whole_counts = int(counts)
    leftover_counts = counts - whole_counts

    # Simulate A/B encoder transitions
    for _ in range(abs(whole_counts)):
        if whole_counts > 0:
            position = (position + 1) % 4
        else:
            position = (position - 1) % 4

        encoder.update(*states[position])

    # Measure speed using the encoder
    measured_speed = encoder.get_speed(20)
    speed_history.append(measured_speed)
    time_history.append((step + 1) * DT)
    power_history.append(power)

    # Print results every half second
    if (step + 1) % 25 == 0:
        print(
            round((step + 1) * DT, 1),
            "|", TARGET_SPEED,
            "|", round(measured_speed, 1),
            "|", round(power, 3)
        )

print("\nSimulation complete!")
print("Total encoder counts:", encoder.count)
print("Total distance (mm):",
      round(encoder.get_distance(), 2))

# Average measured speed over the final 0.5 seconds
final_speeds = speed_history[-25:]
average_speed = sum(final_speeds) / len(final_speeds)

error_percent = (
    abs(TARGET_SPEED - average_speed)
    / TARGET_SPEED * 100
)

print("\nPID Accuracy Results")
print("Target speed:", TARGET_SPEED, "mm/s")
print("Average speed:", round(average_speed, 2), "mm/s")
print("Error:", round(error_percent, 2), "%")

if error_percent <= 5:
    print("PASS: Average speed within 5% of target")
else:
    print("FAIL: Average speed error exceeds 5%")

# Save the simulation data to a CSV file
csv_path = Path(__file__).with_name("speed_log.csv")

with open(csv_path, "w", newline="") as file:
    writer = csv.writer(file)

    writer.writerow([
        "Time (s)",
        "Target Speed (mm/s)",
        "Actual Speed (mm/s)",
        "Motor Power"
    ])

    for i in range(len(speed_history)):
        writer.writerow([
            round(time_history[i], 2),
            TARGET_SPEED,
            round(speed_history[i], 2),
            round(power_history[i], 3)
        ])

print("\nSpeed data saved to speed_log.csv")
