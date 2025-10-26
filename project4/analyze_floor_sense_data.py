import numpy as np
import matplotlib.pyplot as plt

def analyze_data(filename='floor_sensor_data.txt'):
    try:
        # Load data from the file
        times = []
        values = []
        with open(filename, 'r') as f:
            for line in f:
                time, value = map(float, line.strip().split(','))
                times.append(time)
                values.append(value)
        
        # Convert to numpy array for analysis
        values = np.array(values)

        # Compute statistical details
        mean = np.mean(values)
        variance = np.var(values)
        std_dev = np.std(values)
        min_val = np.min(values)
        max_val = np.max(values)

        print(f"Statistical Details for {filename}:")
        print(f"Mean: {mean}")
        print(f"Variance: {variance}")
        print(f"Standard Deviation: {std_dev}")
        print(f"Min Value: {min_val}")
        print(f"Max Value: {max_val}")

        # Plot histogram (frequency graph)
        plt.figure(figsize=(10, 5))
        plt.hist(values, bins=15, range=(90, 160), edgecolor='k', alpha=0.7)
        plt.title('Frequency Distribution of Floor Sensor Values')
        plt.xlabel('Sensor Reading')
        plt.ylabel('Frequency')
        plt.grid()
        plt.xticks(np.arange(90, 161, 10))
        plt.show()

    except FileNotFoundError:
        print(f"Error: File {filename} not found.")
    except Exception as e:
        print(f"Error while analyzing data: {e}")

if __name__ == '__main__':
    analyze_data('floor_sensor_data.txt')
