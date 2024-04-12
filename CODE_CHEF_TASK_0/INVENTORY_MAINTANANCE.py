# Function to maintain the inventory list
def manage_inventory(N, initial_inventory, M, operations):
    inventory = dict()

    for item, quantity in initial_inventory:
        inventory[item] = int(quantity)  # Convert quantity to integer

    output = []
    total_quantity = 0

    for operation in operations:
        op, item, quantity = operation
        if op == "ADD":
            if item in inventory:
                inventory[item] += int(quantity)  # Convert quantity to integer
                output.append(f"UPDATED Item {item}")
            else:
                inventory[item] = int(quantity)  # Convert quantity to integer
                output.append(f"ADDED Item {item}")
        elif op == "DELETE":
            if item not in inventory:
                output.append(f"Item {item} does not exist")
            elif inventory[item] < int(quantity):  # Convert quantity to integer
                output.append(f"Item {item} could not be DELETED")
            else:
                inventory[item] -= int(quantity)  # Convert quantity to integer
                output.append(f"DELETED Item {item}")
        total_quantity = sum(inventory.values())

    output.append(f"Total Items in Inventory: {total_quantity}")
    return output

# Main function
if __name__ == '__main__':
    # Input the number of test cases
    T = int(input())

    # Process each test case
    for _ in range(T):
        # Input the number of items in the lab initially
        N = int(input())

        # Input the initial inventory
        initial_inventory = [input().split() for _ in range(N)]

        # Input the number of operations
        M = int(input())

        # Input the list of operations
        operations = [input().split() for _ in range(M)]

        # Call the manage_inventory function to process the inventory
        result = manage_inventory(N, initial_inventory, M, operations)

        # Print the result for this test case
        for line in result:
            print(line)
