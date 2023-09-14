# Initialize the variables
alpha = 0.2  # Smoothing factor
filtered_value = 10
import time

while True:
    # Read the input value
    input_value = 0

    # If it's the first iteration, initialize the filtered value
    if filtered_value is None:
        filtered_value = input_value
    else:
        # Apply the exponential smoothing formula
        filtered_value = alpha * input_value + (1 - alpha) * filtered_value

    # Use the filtered value for further processing
    print(filtered_value)
    time.sleep(0.1)