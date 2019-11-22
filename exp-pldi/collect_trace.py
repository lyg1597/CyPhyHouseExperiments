import rosbag
import sys

if len(sys.argv) != 2:
    print("Please provide a ROS bag file as input")


def vec3_to_list(v):
    return [v.x, v.y, v.z]


def quat_to_list(v):
    return [v.x, v.y, v.z, v.w]


def pose_to_list(p):
    return vec3_to_list(p.position) + quat_to_list(p.orientation)


def twist_to_list(tw):
    return vec3_to_list(tw.linear) + vec3_to_list(tw.angular)


def bag_to_list(bag):
    sync_dict = {}

    for topic, msg, t in bag.read_messages(
            topics=['/vrpn_client_node/hotdec_car/pose', '/vrpn_client_node/hotdec_car/twist']):

        t_us = (t.to_nsec() // 10000) * 10  # Tolerate 10 micro sec error
        if sync_dict.get(t_us, None) == None:
            sync_dict[t_us] = {}

        if topic == "/vrpn_client_node/hotdec_car/pose":
            sync_dict[t_us]['pose'] = msg.pose
        elif topic == "/vrpn_client_node/hotdec_car/twist":
            sync_dict[t_us]['twist'] = msg.twist
        else:
            raise RuntimeError("Unexpected topic names")

    trace_list = [
        [t] + pose_to_list(state['pose']) + twist_to_list(state['twist'])
        for t, state in sync_dict.items()
        if state.get('pose') and state.get('twist')  # Filter out entry with missing fields
    ]
    return trace_list


import math


def bloatToTube(k, gamma, init_delta_array, trace, dimensions):
    center_trace = trace
    reach_tube = []
    trace_len = len(trace)
    for i in range(trace_len - 1):
        # pdb.set_trace()
        time_interval = center_trace[i + 1][0] - center_trace[0][0]
        lower_rec = [center_trace[i][0]]
        upper_rec = [center_trace[i + 1][0]]

        for dim in range(1, dimensions):
            delta = k[dim - 1] * math.exp(gamma[dim - 1] * time_interval) * init_delta_array[dim - 1]
            upper_rec.append(max(center_trace[i + 1][dim], center_trace[i][dim]) + delta)
            lower_rec.append(min(center_trace[i + 1][dim], center_trace[i][dim]) - delta)
        reach_tube.append(lower_rec)
        reach_tube.append(upper_rec)
    return reach_tube

def A_calc(traces,trace_len):
    trace = traces[0]
    A = []
    for i in range(trace_len-1):
        A.append([-1, -(trace[i+1][0]-trace[0][0])])  # [-1, -(t(i+1)-t0)]

    # write A matrix to file
    # with open('./A.txt', 'w') as write_file:
    #     for i in range(len(A)):
    #         write_file.write(str(A[i])+ '\n')
    return A

def b_calc(trace_len, traces,dim, init_delta):
    #global trace_len
    #global num_traces
    #global init_delta

    b = []

    #print(trace_len)
    #init_delta = abs(traces[0][0][dim] - traces[1][0][dim])

    # check traces number; if too few traces, give up
    num_traces = len(traces)
    if num_traces < 3:
        print("Only have " + str(num_traces-1) + " sample traces! Give up!")
        return None

    # compute b matrix
    for i in range(trace_len-1):   # t = t(i+1)
        maxval = -float('Inf')

        # select any of the two traces and compute their difference in value at t(i+1)
        for j in range(num_traces):
            for k in range(j+1,num_traces):
                trace1 = traces[j]
                trace2 = traces[k]
                #init_delta = max(init_delta,abs(trace1[0][dim]-trace2[0][dim]))
                #print(i)
                #rint(len(trace1))
                #print(len(trace2))
                if abs(trace1[0][dim]-trace2[0][dim]) <= 1e-3:
                    # print("Same trace detected! Algorithm stops!")
                    # print("These two traces are the same trace:")
                    # print("trace",j)
                    # print("trace",k)
                    return 'Same_trace!'
                if trace1[i+1][dim]-trace2[i+1][dim] == 0:
                    val = -float('Inf')
                else:
                    val = math.log(abs(trace1[i+1][dim]-trace2[i+1][dim]))-math.log(abs(trace1[0][dim]-trace2[0][dim]))
                if val > maxval:
                    maxval = val
        b.append(-maxval)



init_del_array = [0.01 for i in range(14)]
k = [1 for i in range(14)]
gamma = [0 for i in range(14)]
dimensions = 14

import math
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import matplotlib.animation as animation
from matplotlib import style
import numpy as np
if __name__ == "__main__":

    bag = rosbag.Bag(sys.argv[1])
    trace = bag_to_list(bag)
    p = bloatToTube(k,gamma,init_del_array,trace,dimensions)
    x = np.array(trace)[:, 1]
    y = np.array(trace)[:, 2]
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)

    trace_plt= ax.plot(x, y, 'k.')
    currentAxis = plt.gca()


    for i in range(0, len(p) - 1, 2):
        w100 = p[i + 1][1] - p[i][1]
        h100 = p[i + 1][2] - p[i][2]

        rect100 = Rectangle((p[i][1], p[i][2]), w100, h100, linewidth=1, edgecolor='c',
                            facecolor='none', alpha=0.5)


        # add the patch to axes
        currentAxis.add_patch(rect100)


    plt.show()
    bag.close()
