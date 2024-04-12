def compute_distance(x1, y1, x2, y2):
    distance = ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
    return round(distance, 2)
if __name__ == '__main__':
    test_cases = int(input())
    for _ in range(test_cases):
        x1, y1, x2, y2 = map(int, input().split())
        distance = compute_distance(x1, y1, x2, y2)
        print(f"Distance: {distance:.2f}")
