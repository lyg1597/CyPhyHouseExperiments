import argparse, socket, time, threading
from collections import OrderedDict 



def update_device ( device_list: dict ):
    '''
    INPUT  - device_list: a dictionary to store all the discoverd device
    OUTPUT - NONE
    FUNCTIONALITY - This function updates the device list 
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
            device_list.update( {i: [info[0], str(address[0]), info[1]]} )
            i +=1
            print( info[0], " from ",address )
        except:
            break
    print("[INFO]: Discover finished")


def execute(address):
    import paramiko

    trans = paramiko.Transport((address, 22))
    trans.connect(username='pi', password='cyphyhouse')
    ssh = paramiko.SSHClient()
    ssh._transport = trans
    stdin, stdout, stderr = ssh.exec_command('./flydrone.bash')
    trans.close()

#-----------------------------------------------------------------------
# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-u", "--url",      required = False,   default = "https://github.com/cyphyhouse/CyPyHous3.git", help="path to the git repo, default to be the ONE")
args = vars(ap.parse_args())

# dictionary to keep track device list
device_list = {}

# Step 1 : Device Discovery 
update_device(device_list)
print("\n ----------------------- Sending CMD -----------------------")
for index, attributes in device_list.items():
    print("sending command to :", attributes[1])
    execute(attributes[1])
print(" -------------------------------------------------------------\n")






