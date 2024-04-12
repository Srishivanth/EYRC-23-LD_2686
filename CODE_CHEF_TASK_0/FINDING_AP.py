# Import reduce module
from functools import reduce

# Function to generate the A.P. series
def generate_AP(a1, d, n):
    AP_series = [a1 + i * d for i in range(n)]
    return AP_series

# Main function
if __name__ == '__main__':
    
    # Take the T (test_cases) input
    test_cases = int(input())
    
    # Iterate through test cases
    for _ in range(test_cases):
        
        # Take the a1, d, and n values as input
        a1, d, n = map(int, input().split())
        
        # Once you have all 3 values, call the generate_AP function to find A.P. series and print it
        AP_series = generate_AP(a1, d, n)
        print(" ".join(map(str, AP_series)))
        
        # Using lambda and map functions, find squares of terms in AP series and print it
        sqr_AP_series = list(map(lambda x: x**2, AP_series))
        print(" ".join(map(str, sqr_AP_series)))
        
        # Using lambda and reduce functions, find the sum of squares of terms in AP series and print it
        sum_sqr_AP_series = reduce(lambda x, y: x + y, sqr_AP_series)
        print(sum_sqr_AP_series)
