""" example code for EVK client """
from struct import unpack_from
import json
import numpy as np
from websocket import create_connection
import math
import sys

from sensor_msgs.msg import PointCloud2
import rospy
from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension,Float32MultiArray

DTYPES = {
    0: np.int8,
    1: np.uint8,
    2: np.int16,
    3: np.uint16,
    4: np.int32,
    5: np.uint32,
    6: np.float32,
    7: np.float64,
}

ASCII_RS = '\u001e'
ASCII_US = '\u001f'



def talk(msg): # send data
      rospy.init_node('talk', anonymous=True)
      pub = rospy.Publisher('/radar_track', String, queue_size=10)
      r = rospy.Rate(10) # 10hz
      rospy.loginfo(msg)
      #msg = Float32MultiArray()
      pub.publish(msg)
      r.sleep()




def to_message(buffer):
    """ parse MatNet messages from JSON / Vayyar internal binary format """
    if isinstance(buffer, str):
        return json.loads(buffer)
    seek = 0
    fields_len = unpack_from('i', buffer, seek + 4)[0]
    fields_split = unpack_from(str(fields_len) + 's', buffer, seek + 8)[0].decode('utf8').split(ASCII_RS)
    msg = {'ID': fields_split[0], 'Payload': dict.fromkeys(fields_split[1].split(ASCII_US))}
    seek += 8 + fields_len
    for key in msg['Payload']:
        seek += np.int32().nbytes
        dtype = DTYPES[(np.frombuffer(buffer, np.int32, 1, seek)).item()]
        seek += np.int32().nbytes
        ndims = (np.frombuffer(buffer, np.int32, 1, seek)).item()
        seek += np.int32().nbytes
        dims = np.frombuffer(buffer, np.int32, ndims, seek)
        seek += ndims * np.int32().nbytes
        data = np.frombuffer(buffer, dtype, np.prod(dims), seek)
        seek += np.prod(dims) * dtype().nbytes
        msg['Payload'][key] = data.reshape(dims) if ndims else data.item()
    return msg


def main():
    # """ connect to server and echoing messages """
    listener = create_connection("ws://10.158.224.89:1234/")
    # retrieve current configuration
    listener.send(json.dumps({
        'Type': 'COMMAND',
        'ID': 'SET_PARAMS',
        'Payload': {
            'MPR.save_dir': r'',                       # Saved records directory
            'MPR.read_from_file': 0.0,                 # 1 – To play records
            'MPR.save_to_file': 0.0,                   # 1 – To save raw data
        }
    }))

    # set outputs for each frame
    listener.send(json.dumps({
        'Type': 'COMMAND',
        'ID': 'SET_OUTPUTS',
        'Payload': {
            # possible binary_outputs: 'pairs', 'freqs', 'I', 'Q', 'inCar_PC_RAW', 'InCarOccupiedSeatsLogical',
              # 'InCarOccupantTypeClass'
            # possible json_outputs: same as the binary outputs + 'inCar_PC_Processed', 'InCarCpdAlarm'
            'binary_outputs': ['Vayyar_InCarProcessedPointCloud'],
            'json_outputs': ['Vayyar_InCarProcessedPointCloud']
        }
    }))

    # start the engine - only if WebGUI isn't present
    listener.send(json.dumps({
        'Type': 'COMMAND',
        'ID': 'START',
        'Payload': {}
    }))

    # request for binary data. Can also request 'JSON_DATA' if 'json_outputs' were specified
    listener.send(json.dumps({'Type': 'QUERY', 'ID': 'BINARY_DATA'}))
    listener.send(json.dumps({'Type': 'QUERY', 'ID': 'JSON_DATA'}))
    print("Running! Waiting for messages...")
    while True:
        buffer = listener.recv()
        data = to_message(buffer)
        #print(data['ID'])
        if data['ID'] == 'BINARY_DATA':
            # data['Payload'] is now available to use
            for index, item in enumerate (data['Payload']):
                #print(item) 
                print(data)
                talk(data['Payload'][item].__str__())
               

            listener.send(json.dumps({'Type': 'QUERY', 'ID': 'BINARY_DATA'}))
        if data['ID'] == 'JSON_DATA':
            for index, item in enumerate (data['Payload']):

                #print(data['Payload'][item])
                talk(data['Payload'][item].__str__())
                #talker.talk(item)
            listener.send(json.dumps({'Type': 'QUERY', 'ID': 'JSON_DATA'}))
        if data['ID'] == 'GET_STATUS':
            print(data['Payload']['status'])
        if data['ID'] == 'SET_PARAMS':
            print("Set parameters:")
            for key in data['Payload']:
                print(key, data['Payload'][key])
    listener.close()


if __name__ == '__main__':
    
    main()
    talk()
  
