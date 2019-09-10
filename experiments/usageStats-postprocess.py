import json
from matplotlib import pyplot as plt
nodes = {}

with open('usageStats.txt', 'r') as f:
    for line in f:
        line_info = line.strip('/').split(',')
        # print line_info
        node = line_info[0]
        if node not in nodes:
            nodes[node] = {}
            nodes[node]['cpu_percent'] = []
            nodes[node]['num_threads'] = []
            nodes[node]['memory_percent'] = []
        nodes[node]['cpu_percent'].append(float(line_info[1].strip()))
        nodes[node]['num_threads'].append(float(line_info[2].strip()))
        nodes[node]['memory_percent'].append(float(line_info[3].strip()))

# Plot 
index = range(0, len(nodes['drone0/waypoint_node']['num_threads']))
plt.plot(index, nodes['drone0/waypoint_node']['num_threads'])
plt.xlabel('Index')
plt.ylabel('Number of Threads')
plt.title('Drone0 Waypoint Node Threads Stats')
plt.show()

    # with open('usageStats.json', 'w') as json_file:
    #     json.dump(nodes, json_file)
