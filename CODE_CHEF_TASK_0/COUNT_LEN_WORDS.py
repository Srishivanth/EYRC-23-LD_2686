T = int(input())
results = []
for _ in range(T):
    input_sentence = input().strip()
    input_sentence = input_sentence[1:]
    words = input_sentence.split()
    word_lengths = ', '.join(str(len(word)) for word in words)
    results.append(word_lengths)
for result in results:
    print(result)
