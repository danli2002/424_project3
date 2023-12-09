import matplotlib.pyplot as plt

# Read the text file and extract the data as a list
with open('speed.txt', 'r') as file:
    content = file.read().strip()  # Read the content and remove leading/trailing whitespaces
    numbers_list = content.split(',')  # Split the content into a list using comma as a delimiter

# Convert the list of strings to a list of integers
numbers_list = [int(num) for num in numbers_list]

# Generating x-values (indices of the list)
x_values = list(range(len(numbers_list)))

# Plotting the data
plt.plot(x_values, numbers_list)
plt.xlabel('X-axis')
plt.ylabel('Y-axis')
plt.title('Line Plot of Y-values')
plt.show()
