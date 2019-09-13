from rqt_top.node_info import NodeInfo

if __name__ == '__main__':
    import sys

    testNode = NodeInfo()
    statFile = open(sys.argv[1], 'w')
    header = ','.join(["node","cpu_percent","num_threads","mem_percent"])
    statFile.write(header+'\n')

    while(True):
        nodes = testNode.get_all_node_fields( ['pid', 'get_cpu_percent', 'get_memory_percent', 'get_num_threads'])

        for node in nodes:
            str1 = str(node['node_name']) + ", " + str(node['cpu_percent']) + ", " + str(node['num_threads']) + ", " + str(node['memory_percent']) + "\n"
            statFile.write(str1)
            
