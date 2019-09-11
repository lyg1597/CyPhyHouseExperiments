from rqt_top.node_info import NodeInfo

if __name__ == '__main__':
    import sys

    testNode = NodeInfo()
    statFile = open(sys.argv[1], 'w')

    while(True):
        nodes = testNode.get_all_node_fields( ['pid', 'get_cpu_percent', 'get_memory_percent', 'get_num_threads'])

        for node in nodes:
            #print(node)
            if(node['node_name'] == '/gazebo_gui'):
                str1 = str(node['node_name']) + ", " + str(node['cpu_percent']) + ", " + str(node['num_threads']) + ", " + str(node['memory_percent']) + "\n"
                statFile.write(str1)
                #print(node['node_name'], "CPU Usage: ", node['cpu_percent'], "Num Threads: ",node['num_threads'], "Mem Usage: ",node['memory_percent'])
            if(node['node_name'] == '/gazebo'):
                str1 = str(node['node_name']) + ", " + str(node['cpu_percent']) + ", " + str(node['num_threads']) + ", " + str(node['memory_percent']) + "\n"
                statFile.write(str1)
            if(node['node_name'] == '/vrpn_client_node'):
                str1 = str(node['node_name']) + ", " + str(node['cpu_percent']) + ", " + str(node['num_threads']) + ", " + str(node['memory_percent']) + "\n"
                statFile.write(str1)
            if(node['node_name'] == '/rosout'):
                str1 = str(node['node_name']) + ", " + str(node['cpu_percent']) + ", " + str(node['num_threads']) + ", " + str(node['memory_percent']) + "\n"
                statFile.write(str1)
            if(node['node_name'] == '/drone0/waypoint_node'):
                str1 = str(node['node_name']) + ", " + str(node['cpu_percent']) + ", " + str(node['num_threads']) + ", " + str(node['memory_percent']) + "\n"
                statFile.write(str1)
            if(node['node_name'] == '/drone1/waypoint_node'):
                str1 = str(node['node_name']) + ", " + str(node['cpu_percent']) + ", " + str(node['num_threads']) + ", " + str(node['memory_percent']) + "\n"
                statFile.write(str1)
            if(node['node_name'] == '/drone2/waypoint_node'):
                str1 = str(node['node_name']) + ", " + str(node['cpu_percent']) + ", " + str(node['num_threads']) + ", " + str(node['memory_percent']) + "\n"
                statFile.write(str1)
            if(node['node_name'] == '/drone3/waypoint_node'):
                str1 = str(node['node_name']) + ", " + str(node['cpu_percent']) + ", " + str(node['num_threads']) + ", " + str(node['memory_percent']) + "\n"
                statFile.write(str1)
            if(node['node_name'] == '/drone4/waypoint_node'):
                str1 = str(node['node_name']) + ", " + str(node['cpu_percent']) + ", " + str(node['num_threads']) + ", " + str(node['memory_percent']) + "\n"
                statFile.write(str1)
            if(node['node_name'] == '/drone5/waypoint_node'):
                str1 = str(node['node_name']) + ", " + str(node['cpu_percent']) + ", " + str(node['num_threads']) + ", " + str(node['memory_percent']) + "\n"
                statFile.write(str1)


            #print("\n")
