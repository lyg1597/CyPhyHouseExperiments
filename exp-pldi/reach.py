import math


def bloatToTube(self, k, gamma, init_delta_array, trace, dimensions):
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


