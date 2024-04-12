# Function to calculate and print the values based on the condition
def calculate_values(n):
    result = []
    for i in range(n):
        if i == 0:
            result.append(str(3))  # Add 3 to i if i is zero
        elif i % 2 == 0:  # Check if i is even
            result.append(str(2 * i))
        else:
            result.append(str(i * i))  # Square of i for odd values
    return ' '.join(result)  # Join the list of values into a space-separated string

# Input the number of test cases
T = int(input())

# Process each test case
for _ in range(T):
    n = int(input())  # Input n for this test case
    result = calculate_values(n)
    print(result)  # Print the calculated values for this test case
