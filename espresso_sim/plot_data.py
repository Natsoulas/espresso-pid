import pandas as pd
import matplotlib.pyplot as plt

# Read data from CSV file
data = pd.read_csv('sim_results/simulation_data.csv')

# Plot temperature over time
plt.figure(figsize=(10, 5))
plt.subplot(2, 1, 1)
plt.plot(data['Time'], data['Temperature'], label='Temperature')
plt.xlabel('Time (s)')
plt.ylabel('Temperature (°C)')
plt.title('Temperature Over Time')
plt.legend()

# Plot control output over time
plt.subplot(2, 1, 2)
plt.plot(data['Time'], data['Control_Output'], label='Control Output')
plt.xlabel('Time (s)')
plt.ylabel('Control Output (V)')
plt.title('Control Output Over Time')
plt.legend()

# Show the plot
plt.tight_layout()
plt.show()
