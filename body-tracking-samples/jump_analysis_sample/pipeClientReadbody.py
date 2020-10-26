
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

clipFrames = 122

buffer = []
currentdata = []

#model = keras.models.load_model('C:\\Users\\Anton\\Desktop\\aks\\Azure-Kinect-Samples-master\\body-tracking-samples\\jump_analysis_sample\\models\\mdl_wts.hdf5')

from GestureRecognitionML.Model import CNN_n_LSTM

model = CNN_n_LSTM.CNN_n_LSTM(lr=0, bs=0, e=0, loadModel=True, split=1, f='splitRecords', path="GestureRecognitionML/")


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
            OldStamp = data[288]
            if(len(buffer) == clipFrames):
                buffer.pop(0)

            buffer.append(data[:-3]) #removes the last 3 elements from the list (x, y ,z)
            rs = np.asarray(buffer)

            if(len(buffer) == clipFrames):
                shape = np.reshape(rs, (rs.shape[0], rs.shape[1]))
                encode = ['jumping jacks', 'sitting arms down', 'sitting crossed arms', 'walking']
                predict = model.predict(shape)[0]


                print('PREDICTION:')
                encodedLabels = []
                for i in range(0, len(encode)):
                    encodedLabels.append((encode[i], predict[i]))

                def sortByCertainty(label):
                    return -label[1]

                encodedLabels.sort(key=sortByCertainty)
                for encode in encodedLabels:
                    print(encode)
                print("*********************************'")
            else:
                print(len(buffer))
                print('buffer not full')

        key = cv2.waitKey(1)
        if key == 27: # Esc key to stop
            break

    print('out of while')
    win32file.CloseHandle(fileHandle)


