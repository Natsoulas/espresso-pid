import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
df = pd.read_csv('espresso_test.csv', header=None)

# Extract column names from the print statement
columns = [
    'elapsedTime',
    'temperature_brew',
    'scaled_output_brew',
    'brewstarted',
    'brew_counter',
    'error_brew',
    'integral_brew',
    'derivative_brew',
    'temperature_boiler',
    'scaled_output_boiler',
    'percentage_boiler',
    'error_boiler',
    'integral_boiler',
    'derivative_boiler',
    'boilerSetpointReached'
]

# Rename columns in the DataFrame
df.columns = columns

# Plot each column against the first column (elapsedTime) on a different figure
for i, column in enumerate(columns[1:]):
    plt.figure()
    plt.plot(df['elapsedTime'], df[column])
    plt.title(column)
    plt.xlabel('Elapsed Time')
    plt.ylabel(column)

plt.show()

