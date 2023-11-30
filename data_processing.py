import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
df = pd.read_csv('espresso_test_with_brew_overshotboiler.csv', header=None)

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

elapsed_time_lower_bound = 1.3575e6
elapsed_time_upper_bound = 1.3605e6

# Plot each column against the first column (elapsedTime) within the specified range
for i, column in enumerate(columns[1:]):
    plt.figure()
    plt.plot(
        df[(df['elapsedTime'] >= elapsed_time_lower_bound) & (df['elapsedTime'] <= elapsed_time_upper_bound)]['elapsedTime'],
        df[(df['elapsedTime'] >= elapsed_time_lower_bound) & (df['elapsedTime'] <= elapsed_time_upper_bound)][column]
    )
    plt.title(column)
    plt.xlabel('Elapsed Time')
    plt.ylabel(column)

plt.show()

