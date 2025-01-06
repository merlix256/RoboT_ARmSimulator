import csv
import random

# Function to generate simple tasks with x, y coordinates
def generate_task_file(file_path, num_tasks=10):
    # Open a new CSV file for writing
    with open(file_path, 'w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=['x_pos', 'y_pos'])
        writer.writeheader()

        # Generate a list of random positions
        for _ in range(num_tasks):
            x_pos = round(random.uniform(-0.2, 0.2), 3)  # Random X position within [-0.2, 0.2]
            y_pos = round(random.uniform(-0.2, 0.2), 3)  # Random Y position within [-0.2, 0.2]
            writer.writerow({'x_pos': x_pos, 'y_pos': y_pos})

    print(f"Generated {num_tasks} tasks in {file_path}")
  
# Generate tasks
generate_task_file('tasks.csv', 10)
