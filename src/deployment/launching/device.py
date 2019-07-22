'''
DANGER: This code contains multiple security loopholes, running in a privatized network is recommanded 
'''
import select, socket, os, threading, subprocess, shutil, datetime, time
from multiprocessing import Process, Queue




# The following two function handles device discover---------------------------------------------------------

def device_query_handler( address: tuple, device_name: str ):  # no returns 
    '''
    INPUT  - address: ip address that the function send the package to 
             device_name: device name
    OUTPUT - NONE
    FUNCATIONALIATY -  This function sends UDP package that contains device name back to the controller 
    '''
    status = "some_status"
    msg = bytes( device_name + ' ' + status ,"utf-8" )
    s = socket.socket( socket.AF_INET, socket.SOCK_DGRAM ) 
    s.sendto( msg, address )
    s.close()

def device_quey_listener(device_name: str):  # no returns
    '''
    INPUT  - device_name: device Name
    OUTPUT - NONE 
    FUNCATIONALIATY:    This function listening on port 60651 for device query
                        Once a query is captured, it will call the device_query_handler to response
    '''

    # Preparing socket 
    port = 60651  
    buffer_size = 64 
    discover_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    discover_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    discover_socket.bind(('0.0.0.0', port))

    # Listening
    while True:
        instruction, address = discover_socket.recvfrom(buffer_size)
        # print( "[DISCOVER]", instruction, " from " , address )
        if( instruction.decode("utf-8") == "INFO" ):
            udp_response = threading.Thread( target=device_query_handler, args=(address, device_name, ) )
            udp_response.start()
        

#-----------------------------------------------------------------------------------------------------------------------
# main
tmp_path = str(os.environ.get('HOME')) + "/tmp"

if os.path.exists(tmp_path):
    shutil.rmtree(tmp_path)
os.makedirs(tmp_path)

device_name = "some_device"
listening_discover         = Process(target=device_quey_listener,args=(device_name, ))
listening_discover.start()
