import random
filename = "c:\\Users\\johns\\Documents\\Johnson\\cmpt 417\\code\\warehouse_instances\\small_1.txt"
f = open(filename, 'r')
# first line: #rows #columns
line = f.readline()
rows, columns = [int(x) for x in line.split(' ')]
rows = int(rows)
columns = int(columns)


mid = columns/2
left_starts = []
left_goals = []
right_starts = []
right_goals = []

for row in range(rows):
    line = f.readline()
    line = line.split(' ')
    # print("columns in line:", len(line)) #125
    for col in range(len(line)):
        if line[col] == 'S':
            if col < mid:
                left_starts.append((row,col))
            else:
                right_starts.append((row,col))
        elif line[col] == 'G':
            if col < mid:
                left_goals.append((row,col))
            else:
                right_goals.append((row,col))

random.shuffle(left_goals)
random.shuffle(right_goals)

for i in range(len(left_starts)):
    start = left_starts[i]
    goal = right_goals[i]
    print(start[0], start[1], goal[0], goal[1])

for i in range(len(right_starts)):
    start = right_starts[i]
    goal = left_goals[i]
    print(start[0], start[1], goal[0], goal[1])
