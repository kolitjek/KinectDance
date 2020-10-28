
# NamedPipe
import win32file
import sys

import keras
import pandas as pd
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

clipFrames = 120

runTest = True

buffer = []
currentdata = []

from GestureRecognitionML.Model.Cnn import cnn
model = cnn(lr=0, bs=0, e=0, loadModel=True, split=1, f='splitRecords', path="GestureRecognitionML/")


class TestData:
    def __init__(self):
        self.data = self.GetData()



    def GetData(self):
        df = pd.read_csv("testVideo.csv", delimiter=";", decimal=".")
        list = df.values.tolist()
        list.pop(0)
        return list

    def GetNextValue(self):
        if len(self.data) == 0:
            return None
        return self.data.pop(0)



if __name__ == "__main__":

    # Create pipe client

    testRun = TestData()

    if not testRun:
        fileHandle = win32file.CreateFile("\\\\.\\pipe\\mynamedpipe",
            win32file.GENERIC_READ | win32file.GENERIC_WRITE,
            0, None,
            win32file.OPEN_EXISTING,
            0, None)

    OldStamp = -1.

    loss = 0
    acc = 0
    labelIndex = 0

    while True:


        if not runTest:
            request_msg = "Request bodyInfo"
            win32file.WriteFile(fileHandle, request_msg.encode())
            inputData = win32file.ReadFile(fileHandle, 1168) #(32*6+1) * 4bytes
            data = np.frombuffer(inputData[1], dtype="float32", count=-1, offset=0)
        else:
            data = testRun.GetNextValue()
            if data == None:
                break

        if(OldStamp != data[288]):
            OldStamp = data[288]
            if(len(buffer) == clipFrames):
                buffer.pop(0)
            elif not testRun:
                buffer.append(data[:-3]) #removes the last 3 elements from the list (x, y ,z)
            else:
                buffer.append(data)


            rs = np.asarray(buffer)

            if(len(buffer) == clipFrames):
                shape = np.reshape(rs, (rs.shape[0], rs.shape[1]))
                print(rs.shape)
                encode = ['fast walk', 'idle', 'jumpiing jacks', 'sitting arms down', 'sitting crossed arms', 'slow walk', 'walking back']
                predict = model.predict(shape, clipFrames, True)[0]


                print('PREDICTION:')
                encodedLabels = []
                for i in range(0, len(predict)):
                    encodedLabels.append((encode[i], predict[i]))


                def sortByCertainty(label):
                    return -label[1]

                encodedLabels.sort(key=sortByCertainty)
                for encode in encodedLabels:
                    print(encode)
                print("*********************************'")
                loss += (1 - predict[labelIndex])
                if encodedLabels[0] == labelIndex:
                    acc += 1
                print(loss)
            else:
                print(len(buffer))
                print('buffer not full')

        key = cv2.waitKey(1)
        if key == 27: # Esc key to stop
            break

    print('out of while')
    win32file.CloseHandle(fileHandle)




