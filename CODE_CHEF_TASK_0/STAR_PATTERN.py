def generate_star_pattern(N):
    for i in range(N):
        stars = N - i 
        row = '*' * stars  
        for j in range(4, stars, 5):
            row = row[:j] + '#' + row[j+1:]      
        print(row)
T = int(input())
for _ in range(T):
    N = int(input())  
    generate_star_pattern(N) 
