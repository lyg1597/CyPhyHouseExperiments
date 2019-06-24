import socket, time, threading, os, multiprocessing,rospy
from collections import OrderedDict 
from std_msgs.msg import String

def update_device ( device_list ):
    '''
    This function updates the device list 
    '''
    buffer_size = 1024
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sender.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sender.sendto(b'INFO', ('<broadcast>', 60651))
    
    sender.settimeout(3.0)

    print("[INFO]: Device query sent!")

    i = 0
    while True: 
        try:
            info, address = sender.recvfrom(buffer_size)
            info = info.decode("utf-8").split(' ')
            device_list.update( { info[0] : str(address[0]) } )
            i +=1
            print( info[0], " from ",address )
        except:
            break
    print("[INFO]: Discover finished")

# -------------------------------------------------
def talker( topic_list, topic_type_list, device_name ):

    os.environ['ROS_MASTER_URI'] = "http://localhost:11311"
    
    rospy.init_node(device_name, anonymous=True)
    pub_list = []
    for i in range( len(topic_list) ):
        topic_type = topic_type_list[i]
        topic      = '/' + device_name + '/' + topic_list[i][1] 

        pub_list.append( rospy.Publisher(topic, topic_type, queue_size = 10 ) )

    while not rospy.is_shutdown():
        print("publishing ---")
        for i in range(len(pub_list)):
            data = topic_list[i][0].get()
            pub_list[i].publish(data)

# -------------------------------------------------
def callback(data, args):
    if( not args[0].full() ):
        args[0].put( data )

def listener( address, topic_list ):
    # subscribe from remote master
    '''
    topic_list = 
    	[ (<multiprocessing.queues.Queue object at 0x109824ef0>, 'chatter0'), 
	  (<multiprocessing.queues.Queue object at 0x109a13860>, 'chatter1'), 
	  (<multiprocessing.queues.Queue object at 0x109a13a90>, 'chatter2')
	], 
    '''
    os.environ['ROS_MASTER_URI'] = "http://" + address + ":11311"
    rospy.init_node('listener', anonymous=True)

    for topic in topic_list:
        rospy.Subscriber(name = topic[1], 
                         data_class =  String, 
                         callback = callback, 
                         callback_args = (topic[0], address)
                        )
    rospy.spin()





# -----------------------------------------------------------------------------------

topic = "chatter"

device_list = {}
update_device(device_list)
print(device_list)

print(os.environ['ROS_IP']) #need to define this first in shell
print(os.environ['ROS_MASTER_URI'])

os.environ['ROS_MASTER_URI'] = "http://localhost:11311"



topic_list      = [ "chatter0", "chatter1", "chatter2"] 
topic_type_list = [ String    , String    , String    ]

data_queue_list = [ [(name, address), [ (multiprocessing.Queue(), topic_name) for topic_name in topic_list ] ]   for name, address in device_list.items() ]
sub_process_list = [ multiprocessing.Process(target = listener, args = ( iterm[0][1], iterm[1], ) ) for iterm in data_queue_list ]
pub_process_list = [ multiprocessing.Process(target = talker, args = (  iterm[1], topic_type_list, iterm[0][0],))  for iterm in  data_queue_list]

for process in sub_process_list :
	process.start()

for process in pub_process_list :
	process.start()










'''
data_queue_list =
        [
                [('device0', '192.168.1.0'), 
                                [(<multiprocessing.queues.Queue object at 0x1052ba7f0>, 'chatter0'), 
                                (<multiprocessing.queues.Queue object at 0x1054ad7b8>, 'chatter1'), 
                                (<multiprocessing.queues.Queue object at 0x1054ad9e8>, 'chatter2')]], 
                
                [('device1', '192.168.1.1'), 
                                [(<multiprocessing.queues.Queue object at 0x1054adc18>, 'chatter0'), 
                                (<multiprocessing.queues.Queue object at 0x1054ade48>, 'chatter1'), 
                                (<multiprocessing.queues.Queue object at 0x1057120b8>, 'chatter2')]], 

                [('device2', '192.168.1.2'), 
                                [(<multiprocessing.queues.Queue object at 0x1057122e8>, 'chatter0'), 
                                (<multiprocessing.queues.Queue object at 0x105712518>, 'chatter1'), 
                                (<multiprocessing.queues.Queue object at 0x105712748>, 'chatter2')]], 

                [('device3', '192.168.1.3'), 
                                [(<multiprocessing.queues.Queue object at 0x105712978>, 'chatter0'), 
                                (<multiprocessing.queues.Queue object at 0x105712ba8>, 'chatter1'), 
                                (<multiprocessing.queues.Queue object at 0x105712dd8>, 'chatter2')]], 

                [('device4', '192.168.1.4'), 
                                [(<multiprocessing.queues.Queue object at 0x10571a048>, 'chatter0'), 
                                (<multiprocessing.queues.Queue object at 0x10571a278>, 'chatter1'), 
                                (<multiprocessing.queues.Queue object at 0x10571a4a8>, 'chatter2')]]

        ]
'''