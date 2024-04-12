# Function to calculate binary equivalent of a decimal number
def dec_to_binary(n, bits=8):
    if bits == 0:
        return ""
    else:
        return dec_to_binary(n // 2, bits - 1) + str(n % 2)

# Main function
if __name__ == '__main__':
    
    # Take the T (test_cases) input
    test_cases = int(input())
    
    # Iterate through test cases
    for _ in range(test_cases):
        
        # Take the n value as input
        n = int(input())
        
        # Calculate the binary equivalent with a length of 8 bits
        binary_num = dec_to_binary(n)
        
        # Print the binary number
        print(binary_num)
