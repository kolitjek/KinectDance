
# NamedPipe
import win32file
import sys

import keras

import numpy as np
import os, os.path

# For visualization
import cv2
import matplotlib.pyplot as plt

# The image size of depth/ir
# Assuming depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED, change it otherwise

# For gray visulization
#MAX_DEPTH_FOR_VIS = 8000.0
#MAX_AB_FOR_VIS = 512.0

clipFrames = 165

buffer = []
currentdata = []

model = keras.models.load_model('C:\\Users\\Anton\\Desktop\\aks\\Azure-Kinect-Samples-master\\body-tracking-samples\\jump_analysis_sample\\models\\mdl_wts.hdf5')



if __name__ == "__main__":

    # Create pipe client

    fileHandle = win32file.CreateFile("\\\\.\\pipe\\mynamedpipe",
        win32file.GENERIC_READ | win32file.GENERIC_WRITE,
        0, None,
        win32file.OPEN_EXISTING,
        0, None)

    OldStamp = -1.

    while True:
        # Send request to pipe server
        request_msg = "Request bodyInfo"
        win32file.WriteFile(fileHandle, request_msg.encode())
        # Read reply data, need to be in same order/size as how you write them in the pipe server in pipe_streaming_example/main.cpp
        inputData = win32file.ReadFile(fileHandle, 1168) #(32*6+1) * 4bytes
        #sys.stdout.write(str(inputData[1]))
        data = np.frombuffer(inputData[1], dtype="float32", count=-1, offset=0)
        #print(len(data))
        if(OldStamp != data[288]):
            #print("%.6f" %data[192])
            OldStamp = data[288]
            if(len(buffer) == clipFrames):
                buffer.pop(0)
                #print(data[0])


            print("X, Y, Z *********************")
            print("%.6f" %data[289])
            print("%.6f" %data[290])
            print("%.6f" %data[291])

            buffer.append(data[:-3]) #removes the last 3 elements from the list (x, y ,z)

            #for x in buffer:
             #   print("%.6f" %x[288])

            rs = np.asarray(buffer)

            #print( np.reshape(rs, (1, rs.shape[0], rs.shape[1])).shape)

            if(len(buffer) == 165 ):
                predict = model.predict(np.reshape(rs, (1, rs.shape[0], rs.shape[1])))
                #pvalues = predict[0]
                #print("%.6f" %pvalues[0])
                #print("%.6f" % pvalues[1])
                #print("%.6f" % pvalues[2])
                print(np.reshape(rs, (1, rs.shape[0], rs.shape[1])))
                print(rs[0][0])
                print("*********************************'")


           # for x in buffer[0]:
            #   print(("%.6f" %x))
            #print("***************************************************************************")


        key = cv2.waitKey(1)
        if key == 27: # Esc key to stop
            break 

    win32file.CloseHandle(fileHandle)


