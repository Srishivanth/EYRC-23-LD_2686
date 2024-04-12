# Function to find students with maximum scores
def find_max_score_students(student_data):
    max_score = -1
    max_score_students = []
    
    # Find the maximum score
    for student in student_data:
        _, score = student
        if score > max_score:
            max_score = score
            max_score_students = [student[0]]
        elif score == max_score:
            max_score_students.append(student[0])
    
    # Sort the names in ascending alphabetical order
    max_score_students.sort()
    
    return max_score_students

# Main function
if __name__ == "__main__":
    # Input the number of test cases
    t = int(input())
    
    # Iterate through each test case
    for _ in range(t):
        # Input the number of students
        n = int(input())
        
        # Initialize a list to store student data as (name, score) pairs
        student_data = []
        
        # Input student names and scores
        for _ in range(n):
            name, score = input().split()
            student_data.append((name, float(score)))  # Convert score to float
        
        # Find students with maximum scores
        max_score_students = find_max_score_students(student_data)
        
        # Print the names of students with maximum scores
        for student in max_score_students:
            print(student)
