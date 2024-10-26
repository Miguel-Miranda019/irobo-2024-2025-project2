import csv

# Initialize an empty list to store the data
data = []

# Open and read the CSV file
with open('/home/ir-labs/catkin_ws/src/rrt_planner/csv/rrt_planer_metrics_03.csv', mode='r') as file:
    csv_reader = csv.reader(file)

    # Loop through each row and add to the list
    for row in csv_reader:
        data.append(row)

data.pop(0)
time = 0.0
old_time = 0.0
lowest_time = 100
highest_time = 0

for row in data:
    if float(row[4]) - old_time < lowest_time:
        lowest_time = float(row[4]) - old_time
    if float(row[4]) - old_time > highest_time:
        highest_time = float(row[4]) - old_time
        if highest_time > 0.5:
            print(row)
    time += float(row[4]) - old_time
    old_time = float(row[4])

print("average time per execution :", time / len(data))
print("lowest time per execution :", lowest_time)
print("highest time per execution :", highest_time)

all_nodes = 0
lowest_nodes = 10000
highest_nodes = 0

for row in data:
    nodes = int(row[1])
    all_nodes += nodes
    if nodes < lowest_nodes:
        lowest_nodes = nodes
    if nodes > highest_nodes:
        highest_nodes = nodes

print("average attempted nodes per execution :", all_nodes / len(data))
print("lowest attempted nodes per execution :", lowest_nodes)
print("highest attempted nodes per execution :", highest_nodes)

last_win = 0
average_nodes_path = 0
lowest_nodes_path = 10000
highest_nodes_path = 0
count = 0

for row in data:
    if int(row[2]) > last_win and int(row[7]) == 1:
        last_win = int(row[2])
        nodes = int(row[6])
        count += 1
        average_nodes_path += nodes
        if nodes > highest_nodes_path:
            highest_nodes_path = nodes
        if nodes < lowest_nodes_path:
            lowest_nodes_path = nodes

print("average nodes in path per victory :", average_nodes_path / count)
print("lowest nodes in path per victory :", lowest_nodes_path)
print("highest nodes in path per victory :", highest_nodes_path)

average_nodes_tree = 0
lowest_nodes_tree = 10000
highest_nodes_tree = 0

for row in data:
    nodes = int(row[5])
    average_nodes_tree += nodes
    if nodes > highest_nodes_tree:
        highest_nodes_tree = nodes
    if nodes < lowest_nodes_tree:
        lowest_nodes_tree = nodes

print("average nodes per tree :", average_nodes_tree / len(data))
print("lowest nodes per tree :", lowest_nodes_tree)
print("highest nodes per tree :", highest_nodes_tree)

average_block_path = 0
lowest_block_path = 10000
highest_block_path = 0

for row in data:
    block = int(row[3])
    average_block_path += block
    if block > highest_block_path:
        highest_block_path = block
    if block < lowest_block_path:
        lowest_block_path = block

print("average blocks per attempt :", average_block_path / len(data))
print("lowest blocks per attempt :", lowest_block_path)
print("highest blocks per attempt :", highest_block_path)

print("average %blocks per attempt :", average_block_path / all_nodes)
print("lowest %blocks per attempt :", lowest_block_path / lowest_nodes)
print("highest %blocks per attempt :", highest_block_path / highest_nodes)

count_dif = 0
amcl_dif = 0
rrt_dif = 0
real_dif = 0
for row in data:
    real = float(row[8])
    real_dif += real
    amcl = float(row[9])
    rrt = float(row[10])
    if real < amcl:
        #print(f"{real} : {amcl} : {rrt}")
        amcl_dif = amcl - real
        rrt_dif = rrt - real
        count_dif += 1
print("average amcl extra distance :", amcl_dif / count_dif)
print("average rrt extra distance :", rrt_dif / count_dif)
print("average distance of each pat:", real_dif / count_dif)