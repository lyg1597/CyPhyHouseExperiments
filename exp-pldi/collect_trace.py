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

    for topic, msg, t in bag.read_messages(topics=['/vrpn_client_node/hotdec_car/pose', '/vrpn_client_node/hotdec_car/twist']):

        t_us = (t.to_nsec() // 10000) * 10 # Tolerate 10 micro sec error
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
            if state.get('pose') and state.get('twist') # Filter out entry with missing fields
    ]
    return trace_list


if __name__ == "__main__":
    bag = rosbag.Bag(sys.argv[1])

    print(bag_to_list(bag))
    bag.close()

