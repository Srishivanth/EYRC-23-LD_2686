# Function to perform operations on the list
def perform_operations(arr):
    # Print the list in reverse order
    print(" ".join(map(str, arr[::-1])))

    # Print every third number with 3 added to it
    third_nums = [arr[i] + 3 for i in range(len(arr)) if ((i % 3 == 0)&(i != 0))]
    print(" ".join(map(str, third_nums)))

    # Print every fifth number with 7 subtracted from it
    fifth_nums = [arr[i] - 7 for i in range(len(arr)) if ((i % 5 == 0) & (i != 0))]
    print(" ".join(map(str, fifth_nums)))

    # Calculate and print the sum of numbers with an index between 3 and 7 (inclusive)
    sum_nums = sum(arr[3:8])
    print(sum_nums)

# Main function
if __name__ == "__main__":
    t = int(input())
    for _ in range(t):
        n = int(input())
        arr = list(map(int, input().split()))
        perform_operations(arr)
