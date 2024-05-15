import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file into a DataFrame
df = pd.read_csv("/tmp/plotting_data.csv", header=None, names=["Timestamp", "steeringWheelAngle", "actual_steering"])

# Convert the Timestamp column to datetime format (assuming microseconds)
df["Timestamp"] = pd.to_datetime(df["Timestamp"], unit="us")

# Initialize variables for percentage correct calculation
total_nonzero_frames = 0
total_correct = 0

# Iterate through each row in the DataFrame
for index, row in df.iterrows():
    # Check if the actual steering is non-zero
    if abs(row["actual_steering"]) > 0.0001:
        total_nonzero_frames += 1
        # Check if the absolute difference between steeringWheelAngle and actual_steering
        # is within 25% of the absolute value of actual_steering
        if abs(row["steeringWheelAngle"] - row["actual_steering"]) <= 0.25 * abs(row["actual_steering"]):
            total_correct += 1

# Calculate percentage correct
if total_nonzero_frames > 0:
    percentage_correct = (total_correct / total_nonzero_frames) * 100
else:
    percentage_correct = 0

print("Percentage Correct:", percentage_correct, "%")

# Plot the data
plt.figure(figsize=(10, 6))
plt.plot(df["Timestamp"], df["steeringWheelAngle"], label="Steering Wheel Angle")
plt.plot(df["Timestamp"], df["actual_steering"], label="Actual Steering")
plt.xlabel("Timestamp")
plt.ylabel("Value")
plt.title("Steering Wheel Angle vs Actual Steering\nPercentage Correct: {:.2f}%".format(percentage_correct))
plt.legend()
plt.grid(True)
plt.show()

plt.savefig("steering_plot.png")

