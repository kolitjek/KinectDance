// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#define _CRT_SECURE_NO_DEPRECATE
#include <array>
#include <iostream>
#include <map>
#include <vector>
#include <filesystem>
#include <windows.h>

#include <k4a/k4a.h>
#include <k4abt.h>
#include <iostream>
#include <iomanip>


#include <BodyTrackingHelpers.h>
#include <Utilities.h>
#include <Window3dWrapper.h>
#include <fstream>
#include <utility> // std::pair
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream

#include "JumpEvaluator.h"
#include <chrono>
#include <time.h>


using namespace std::chrono;
namespace fs = std::filesystem;
void PrintAppUsage()
{
    printf("\n");
    printf(" Basic Usage:\n\n");
    printf(" 1. Make sure you place the camera parallel to the floor and there is only one person in the scene.\n");
    printf(" 2. Press 'K' to enable handsdetector. This allows you to start a record when lifting your arms above your head and doing it again will end record\n");
    printf(" 3. Use 'R' to start a record. Press it again to stop record\n");
    printf(" 4. When you have stopped record, press s to save it and follow instructions.\n");
    printf(" 5. Press 'Q' or 'ESC' to exit program, this is important because it turns off the camera\n");
    printf(" 6. Press 'I' to show pointCloud. This will be stopped if recording for performance reasons\n");
    printf("7. Press 'M' to stream to Python Pipe");

    printf("\n");
}

#define BUFSIZE 512
// Buffer size for writing frame data to pipe
// Assuming depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED, change it otherwise
// Assuming streaming 2 channels: depth & ab, change it otherwise
#define RESPOND_BUFFER 772 // (32 * 6 +1 ) * 4 = 772 bytes


DWORD WINAPI InstanceThread(LPVOID);
VOID GetAnswerToRequest(LPTSTR, LPTSTR, LPDWORD);

// Create pipe server
BOOL fConnected = FALSE;
DWORD dwThreadId = 0;
HANDLE hPipe = INVALID_HANDLE_VALUE, hThread = NULL;
LPTSTR lpszPipename = const_cast<LPSTR>(TEXT("\\\\.\\pipe\\mynamedpipe"));


// Global State and Key Process Function
bool s_isRunning = true;

std::vector<k4abt_body_t> m_listOfBodyPositions;
std::vector<float> m_framesTimestampInUsec;

float streamData[193]; //193 datapoints pr frame

const int m_defaultWindowWidth = 640;
const int m_defaultWindowHeight = 576;

std::string pathString;

bool m_reviewWindowIsRunning = false;
bool isRecording = false;
bool quit = false;


bool useHandsToRecordOn = false;

bool runOrLoadChosed = false;

bool saveRecord = false;

bool stream = false;

float oldStamp = -1;

bool streamConnected = false;

bool showPointCloud = false;

bool m_previousHandsAreRaised = false;

HandRaisedDetector m_handRaisedDetector;


int64_t ProcessKey(void* /*context*/, int key)
{
    // https://www.glfw.org/docs/latest/group__keys.html
    switch (key)
    {
        // Quit
    case GLFW_KEY_ESCAPE:
        s_isRunning = false;
        m_reviewWindowIsRunning = false;
        quit = true;
        break;

    case GLFW_KEY_H:
        PrintAppUsage();
        break;

    case GLFW_KEY_S:
        if (s_isRunning == false) {
            m_reviewWindowIsRunning = false;
            saveRecord = true;
        }
        break;

    case GLFW_KEY_I:
        showPointCloud = true;
        break;

    
    case GLFW_KEY_K:
        if (useHandsToRecordOn) {
            useHandsToRecordOn = false;
            printf("Use hands to stop record off \n");
        }
        else {
            useHandsToRecordOn = true;
            printf("Use hands to start record on \n");
        }
        break;

    case GLFW_KEY_Q:
        s_isRunning = false;
        quit = true;
        m_reviewWindowIsRunning = false;
        break;

    case GLFW_KEY_M:
        stream = true;
        printf("Streaming...");
        break;


    case GLFW_KEY_R:
        if (!isRecording) {
            printf("Started new record \n");
            isRecording = true;
        }
        else if (isRecording)
        {
            printf("Stopped recording: press s to save clip \n");
            s_isRunning = false;
        }
        break;

    }
    return 1;
}

float CalcVelGivenTwoPointsAndTime(float x1, float y1, float z1, float time1, float x2, float y2, float z2, float time2) {

    float dist = sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2) + pow((z2 - z1), 2));
    
    float time = time2 - time1;

   // printf("dist: %f, time: %f, vel: %f \n", dist, time, (dist / time));

    return (dist / time)*1000; //meter/second

    

}

int64_t ReviewWindowCloseCallbackMain(void* context)
{

    bool* running = (bool*)context;
    *running = false;
    return 1;
}

void Load_csv(std::string filename) {

    // Create an input filestream
    std::ifstream myFile(filename);
    std::string line, colname;
    float val;

    printf("\nLoading from CSV... \n");

    // Read data, line by line
    while (std::getline(myFile, line))
    {
        // Create a stringstream of the current line
        std::stringstream ss(line);

        // Keep track of the current column index
        int colIdx = 0;

        // Extract each integer

        // Read data, line by line
        while (std::getline(myFile, line))
        {
            // Create a stringstream of the current line
            std::stringstream ss(line);

            // Keep track of the current column index
            int colIdx = 0;
            k4abt_body_t body;
            std::vector<float> vals;
            body.id = 0;


            // Extract each integer //225
            while (ss >> val) {

                // Add the current integer to the 'colIdx' column's values vector
               // printf("%f \n", val);

                if (!(colIdx == 3) && !(colIdx == 7))
                     vals.push_back(val);

                // If the next token is a comma, ignore it and move on
                if (ss.peek() == ';') ss.ignore();

                // Increment the column index
                colIdx++;
                if (colIdx == 9)
                    colIdx = 0;
            }
            for (int i = 0; i < 32; i++) {
                k4a_float3_t pos;
                k4a_quaternion_t orientation;
                k4abt_joint_confidence_level_t confidence;

                pos.v[0] = vals[7 * i ];
                pos.v[1] = vals[7 * i + 1];
                pos.v[2] = vals[7 * i + 2];

                orientation.v[0] = vals[7 * i + 3];
                orientation.v[1] = vals[7 * i + 4];
                orientation.v[2] = vals[7 * i + 5];

                confidence = (k4abt_joint_confidence_level_t)vals[7 * i + 6];

                body.skeleton.joints[i].position = pos;
                body.skeleton.joints[i].orientation = orientation;
                body.skeleton.joints[i].confidence_level = confidence;
            }
            m_listOfBodyPositions.push_back(body);
            m_framesTimestampInUsec.push_back(vals[vals.size() - 1]);
        }
    }
}

void write_csv(std::string path, std::vector<std::pair<std::string, std::vector<float>>> dataset){
    // Create an output filestream object
    std::ofstream myFile(path);
  
    // Send column names to the stream

    for (int j = 0; j < dataset.size(); ++j)
    {
        myFile << std::fixed << std::setprecision(6) << dataset.at(j).first;
        if (j != dataset.size() - 1) myFile << ";"; // No comma at end of line
    }
    myFile << "\n";

    // Send data to the stream
    for (int i = 0; i < dataset.at(0).second.size(); ++i)
    {
        for (int j = 0; j < dataset.size(); ++j)
        {
            myFile << std::fixed << std::setprecision(6) << dataset.at(j).second.at(i);
            if (j != dataset.size() - 1) myFile << ";"; // No comma at end of line
        }
        myFile << "\n";
    }

    // Close the file
    myFile.close();
}


int64_t CloseCallback(void* /*context*/)
{
    s_isRunning = false;
    return 1;
}



//missing closecallback
//missing floorrendering atm  k4a_float3_t standingPosition (parameter)
void CreateRenderWindow(
    Window3dWrapper& window,
    std::string windowName,
    const k4abt_body_t& body,
    int windowIndex)
{
    window.Create(windowName.c_str(), K4A_DEPTH_MODE_WFOV_2X2BINNED, m_defaultWindowWidth, m_defaultWindowHeight);
    window.SetCloseCallback( ReviewWindowCloseCallbackMain, &m_reviewWindowIsRunning);
    window.AddBody(body, g_bodyColors[0]);
   // window.SetFloorRendering(true, standingPosition.v[0] / 1000.f, standingPosition.v[1] / 1000.f, standingPosition.v[2] / 1000.f);

    int xPos = windowIndex * m_defaultWindowWidth;
    int yPos = 100;
    window.SetWindowPosition(xPos, yPos);
}



void LoadFile() {

    // CalcVelGivenTwoPointsAndTime(0, 0, 0, 0, 1000, 1000, 1000, 1000);

    const int numberOfFloatsPrJoint = 225;
    //                            vector pos  orientation                     timestamp
    float str[numberOfFloatsPrJoint]; //32 vectors (body) *( ( 3(x,y,z) + 3*(x,y,z) + 1(confidence) )+ 1 (timestamp)  32 (3 * 3 + 1) +1 = 225 

    int fSize = 0;
    char input[200];

    printf("\n Load a file... \n \n Files to choose: \n");

    for (const auto& entry : fs::directory_iterator(pathString)) {

        std::string s = entry.path().u8string();
        int pos = s.find_last_of('\\');
        std::string ss = s.substr(pos + 1);
        std::string ssFinal = ss.substr(0, ss.length()-4);

        std::cout << ssFinal << std::endl;
    }

    printf("\nEnter a file:   ");
    scanf("%s", input);

    Load_csv(std::string(pathString + input + ".csv").c_str());
    /*
    
    FILE* ptr = fopen(( std::string (pathString + input + ".txt").c_str()), "r");
    if (ptr == NULL)
    {
        printf("no such file.");       
    }
    int counter = 0;

    while (EOF != fscanf(ptr, "%f,", &str[counter])) {
        counter++;

        if (counter == numberOfFloatsPrJoint) {
           
            k4abt_body_t body;
            body.id = 0; //always 0 cause of hardcoding smileyface
            counter = 0;
                     
            m_framesTimestampInUsec.push_back(str[0]);

            for (int i = 0; i < 32; i++) {               
                    k4a_float3_t pos;
                    k4a_quaternion_t orientation;
                    k4abt_joint_confidence_level_t confidence;

                    pos.v[0] = str[7*i+1];
                    pos.v[1] = str[7*i+2];
                    pos.v[2] = str[7*i+3];

                    orientation.v[0] = str[7 * i + 4];
                    orientation.v[1] = str[7 * i + 5];
                    orientation.v[2] = str[7 * i + 6];

                    confidence = (k4abt_joint_confidence_level_t) str[7 * i + 7];

                    body.skeleton.joints[i].position = pos;       
                    body.skeleton.joints[i].orientation = orientation;
                    body.skeleton.joints[i].confidence_level = confidence;
            }
            m_listOfBodyPositions.push_back(body);
        }
    }*/
    //fclose(ptr);   
}

void Stream()
{
    printf(TEXT("\nPipe Server: Main thread awaiting client connection on %s\n"), lpszPipename);
    hPipe = CreateNamedPipe(lpszPipename,               // pipe name
        PIPE_ACCESS_DUPLEX,         // read/write access
        PIPE_TYPE_MESSAGE |         // message type pipe
        PIPE_READMODE_MESSAGE | // message-read mode
        PIPE_WAIT,              // blocking mode
        PIPE_UNLIMITED_INSTANCES,   // max. instances
        RESPOND_BUFFER,              // output buffer size
        RESPOND_BUFFER,              // input buffer size
        0,                          // client time-out
        NULL);                      // default security attribute

    if (hPipe == INVALID_HANDLE_VALUE)
    {
        printf(TEXT("CreateNamedPipe failed, GLE=%d.\n"), GetLastError());
    }

    // Wait for the client to connect; if it succeeds,
    // the function returns a nonzero value. If the function
    // returns zero, GetLastError returns ERROR_PIPE_CONNECTED.

    fConnected = ConnectNamedPipe(hPipe, NULL) ? TRUE : (GetLastError() == ERROR_PIPE_CONNECTED);

    if (fConnected)
    {
        printf("Client connected, creating a processing thread.\n");

        // Create a thread for this client.
        hThread = CreateThread(NULL,           // no security attribute
            0,              // default stack size
            InstanceThread, // thread proc
            (LPVOID)hPipe,  // thread parameter
            0,              // not suspended
            &dwThreadId);   // returns thread ID

        if (hThread == NULL)
        {
            printf(TEXT("CreateThread failed, GLE=%d.\n"), GetLastError());
        }
        else
            CloseHandle(hThread);
    }
    else
        // The client could not connect, so close the pipe.
        CloseHandle(hPipe);

    printf("called again********************************************************************************************************");
}



DWORD WINAPI InstanceThread(LPVOID lpvParam)
// This routine is a thread processing function to read from and reply to a client
// via the open pipe connection passed from the main loop. Note this allows
// the main loop to continue executing, potentially creating more threads of
// of this procedure to run concurrently, depending on the number of incoming
// client connections.
{
    HANDLE hHeap = GetProcessHeap();
    TCHAR* pchRequest = (TCHAR*)HeapAlloc(hHeap, 0, BUFSIZE * sizeof(TCHAR));
    TCHAR* pchReply = (TCHAR*)HeapAlloc(hHeap, 0, RESPOND_BUFFER);

    DWORD cbBytesRead = 0, cbReplyBytes = 0, cbWritten = 0;
    BOOL fSuccess = FALSE;
    HANDLE hPipe = NULL;

    // Do some extra error checking since the app will keep running even if this
    // thread fails.

    if (lpvParam == NULL)
    {
        printf("\nERROR - Pipe Server Failure:\n");
        printf("   InstanceThread got an unexpected NULL value in lpvParam.\n");
        printf("   InstanceThread exitting.\n");
        if (pchReply != NULL)
            HeapFree(hHeap, 0, pchReply);
        if (pchRequest != NULL)
            HeapFree(hHeap, 0, pchRequest);
        return (DWORD)-1;
    }

    if (pchRequest == NULL)
    {
        printf("\nERROR - Pipe Server Failure:\n");
        printf("   InstanceThread got an unexpected NULL heap allocation.\n");
        printf("   InstanceThread exitting.\n");
        if (pchReply != NULL)
            HeapFree(hHeap, 0, pchReply);
        return (DWORD)-1;
    }

    if (pchReply == NULL)
    {
        printf("\nERROR - Pipe Server Failure:\n");
        printf("   InstanceThread got an unexpected NULL heap allocation.\n");
        printf("   InstanceThread exitting.\n");
        if (pchRequest != NULL)
            HeapFree(hHeap, 0, pchRequest);
        return (DWORD)-1;
    }

    // Print verbose messages. In production code, this should be for debugging only.
    printf("InstanceThread created, receiving and processing messages.\n");

    // The thread's parameter is a handle to a pipe object instance.

    hPipe = (HANDLE)lpvParam;

    // Loop until done reading
    while (1)
    {
        // Read client requests from the pipe. This simplistic code only allows messages
        // up to BUFSIZE characters in length.
        fSuccess = ReadFile(hPipe,                   // handle to pipe
            pchRequest,              // buffer to receive data
            BUFSIZE * sizeof(TCHAR), // size of buffer
            &cbBytesRead,            // number of bytes read
            NULL);                   // not overlapped I/O

        if (!fSuccess || cbBytesRead == 0)
        {
            if (GetLastError() == ERROR_BROKEN_PIPE)
            {
                printf(TEXT("InstanceThread: client disconnected.\n"), GetLastError());
            }
            else
            {
                printf(TEXT("InstanceThread ReadFile failed, GLE=%d.\n"), GetLastError());
            }
            break;
        }

        // Process the incoming message.
        GetAnswerToRequest(pchRequest, pchReply, &cbReplyBytes);

        // Write the reply to the pipe.
        fSuccess = WriteFile(hPipe,        // handle to pipe
            pchReply,     // buffer to write from
            cbReplyBytes, // number of bytes to write
            &cbWritten,   // number of bytes written
            NULL);        // not overlapped I/O

        if (!fSuccess || cbReplyBytes != cbWritten)
        {
            printf(TEXT("InstanceThread WriteFile failed, GLE=%d.\n"), GetLastError());
            break;
        }
    }

    // Flush the pipe to allow the client to read the pipe's contents
    // before disconnecting. Then disconnect the pipe, and close the
    // handle to this pipe instance.

    FlushFileBuffers(hPipe);
    DisconnectNamedPipe(hPipe);
    CloseHandle(hPipe);

    HeapFree(hHeap, 0, pchRequest);
    HeapFree(hHeap, 0, pchReply);

    printf("InstanceThread exitting.\n");
    return 1;
}

VOID GetAnswerToRequest(LPTSTR pchRequest,
    LPTSTR pchReply,
    LPDWORD pchBytes)
    // This routine is a simple function to print the client request to the console
    // and populate the reply buffer with a default data string. This is where you
    // would put the actual client request processing code that runs in the context
    // of an instance thread. Keep in mind the main thread will continue to wait for
    // and receive other client connections while the instance thread is working.
{
    //printf(TEXT("Client Request String:\"%s\"\n"), pchRequest);

    // Check the outgoing message to make sure it's not too long for the buffer.

        // Probe for a IR16 and depth image

    if (oldStamp != streamData[192]) {
        // Write depth image data to reply
        memcpy(&pchReply[0], &(streamData[0]), 772);
        oldStamp = streamData[192];
        printf("%f \n", oldStamp);

    }
    else
    {
        //printf(" | Ir16 or Depth None                    ");
        *pchBytes = 0;
        pchReply[0] = 0;
        //printf("StringCchCopy failed, no outgoing message.\n");
    }

    *pchBytes = 772;
    //printf("streamdata size: %d \n", streamData.size());



}


int main()
{
    char rOrL;

    pathString = fs::current_path().u8string().substr(0, fs::current_path().u8string().length() - 15) + "\\records\\";

    printf("Enter 'r' to run or l to load file: ");
    scanf("%c", &rOrL);

    if (rOrL != 'r' && rOrL != 'l') {
        return -1;
    }

    if (rOrL == 'l') {
        s_isRunning = false;
        LoadFile();
    }
    PrintAppUsage();

    k4a_device_t device = nullptr;
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");

    // Start camera. Make sure depth camera is enabled.
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    deviceConfig.camera_fps = K4A_FRAMES_PER_SECOND_30;
    VERIFY(k4a_device_start_cameras(device, &deviceConfig), "Start K4A cameras failed!");

    // Get calibration information
    k4a_calibration_t sensorCalibration;
    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration),
        "Get depth camera calibration failed!");

    // Create Body Tracker
    k4abt_tracker_t tracker = nullptr;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    VERIFY(k4abt_tracker_create(&sensorCalibration, tracker_config, &tracker), "Body tracker initialization failed!");

    // Initialize the 3d window controller
    Window3dWrapper window3d;
    window3d.Create("3D Visualization", sensorCalibration);
    window3d.SetCloseCallback(CloseCallback);
    window3d.SetKeyCallback(ProcessKey);

    Window3dWrapper m_window3dReplay;
    m_window3dReplay.SetCloseCallback(CloseCallback);
    m_window3dReplay.SetKeyCallback(ProcessKey);
    
    bool isReplayWindowCreated = false;

    // Initialize the jump evaluator
   // JumpEvaluator jumpEvaluator;

    clock_t before;
    clock_t difference;
    bool runOnce = false;
    int fpscounter = 0;

    while (s_isRunning)
    {
        k4a_capture_t sensorCapture = nullptr;
        k4a_wait_result_t getCaptureResult = k4a_device_get_capture(device, &sensorCapture, 0); // timeout_in_ms is set to 0

        if (getCaptureResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            // timeout_in_ms is set to 0. Return immediately no matter whether the sensorCapture is successfully added
            // to the queue or not.
            k4a_wait_result_t queueCaptureResult = k4abt_tracker_enqueue_capture(tracker, sensorCapture, 0);

            // Release the sensor capture once it is no longer needed.
            k4a_capture_release(sensorCapture);

            if (queueCaptureResult == K4A_WAIT_RESULT_FAILED)
            {
                std::cout << "Error! Add capture to tracker process queue failed!" << std::endl;
                break;
            }
        }
        else if (getCaptureResult != K4A_WAIT_RESULT_TIMEOUT)
        {
            std::cout << "Get depth capture returned error: " << getCaptureResult << std::endl;
            break;
        }


        // Pop Result from Body Tracker
        k4abt_frame_t bodyFrame = nullptr;
        k4a_wait_result_t popFrameResult = k4abt_tracker_pop_result(tracker, &bodyFrame, 0); // timeout_in_ms is set to 0
        if (popFrameResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            /************* Successfully get a body tracking result, process the result here ***************/

            // Obtain original capture that generates the body tracking result
            k4a_capture_t originalCapture = k4abt_frame_get_capture(bodyFrame);

#pragma region Jump Analysis

            // Add new body tracking result to the jump evaluator
            const size_t JumpEvaluationBodyIndex = 0; // For simplicity, only run jump evaluation on body 0
            if (k4abt_frame_get_num_bodies(bodyFrame) > 0)
            {
                k4abt_body_t body;
                VERIFY(k4abt_frame_get_body_skeleton(bodyFrame, JumpEvaluationBodyIndex, &body.skeleton), "Get skeleton from body frame failed!");
                body.id = k4abt_frame_get_body_id(bodyFrame, JumpEvaluationBodyIndex);
             
                if (!isRecording) {
                    //float dist = sqrt(pow(body.skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.v[0], 2) + pow(body.skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.v[1], 2) + pow(body.skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.v[2], 2));
                    //printf("Distance: %f meters \n", dist / 1000);
                }

                uint64_t timestampUsec = k4abt_frame_get_device_timestamp_usec(bodyFrame);

                if (stream) {

                    for (int k = 0; k < 32; k++) {
                        streamData[k * 6  ] = body.skeleton.joints[k].position.v[0];
                        streamData[k * 6+1] = body.skeleton.joints[k].position.v[1];
                        streamData[k * 6+2] = body.skeleton.joints[k].position.v[2];
                        streamData[k * 6+3] = body.skeleton.joints[k].orientation.v[0];
                        streamData[k * 6+4] = body.skeleton.joints[k].orientation.v[1];
                        streamData[k * 6+5] = body.skeleton.joints[k].orientation.v[2];
                    }
                    streamData[192] = timestampUsec;    

                }            

               if (isRecording) {

                    fpscounter++;


                    if (!runOnce) {
                        before = clock();
                        runOnce = true;
                    }

                    difference = clock() - before;

                    if ((difference / CLOCKS_PER_SEC) >= 1) {
                        printf("fps: %d\n", fpscounter);
                        before = clock();
                        fpscounter = 0;
                    }

                       // printf(" frames in 6 sec %d:  , frames pr sec: %d \n",m_framesTimestampInUsec.size()+1, (m_framesTimestampInUsec.size()+1)/6 );

                        //body.skeleton.joints[K4ABT_JOINT_SPINE_CHEST];
                        //float dist = sqrt(pow(body.skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.v[0], 2) + pow(body.skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.v[1], 2) + pow(body.skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position.v[2], 2));
                        //printf("Distance: %f meters", dist/1000);
                   // }
                    
                        m_listOfBodyPositions.push_back(body);
                        m_framesTimestampInUsec.push_back(static_cast<float>(timestampUsec));
                    
                }

#pragma region Hand Raise Detector
            // Update hand raise detector data
                if (useHandsToRecordOn) {
                    m_handRaisedDetector.UpdateData(body, timestampUsec);

                    // Use hand raise detector to decide whether we should initialize/end a jump session
                    bool handsAreRaised = m_handRaisedDetector.AreBothHandsRaised();
                    if (!m_previousHandsAreRaised && handsAreRaised)
                    {
                        
                        if (isRecording) {
                            printf("\nStopped record, press 's' to save or q to quit \n");
                            s_isRunning = false;
                            //isRecording = false;
                        }
                        else {
                            isRecording = true;
                            printf("Started record \n");
                        }
                    }
                    m_previousHandsAreRaised = handsAreRaised;
                }
#pragma endregion
            }

#pragma endregion
            
            // Visualize point cloud
            k4a_image_t depthImage = k4a_capture_get_depth_image(originalCapture);
            if (!isRecording && showPointCloud) {
                window3d.UpdatePointClouds(depthImage);
            }

            // Visualize the skeleton data
            window3d.CleanJointsAndBones();
            uint32_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);
            for (uint32_t i = 0; i < numBodies; i++)
            {
            k4abt_body_t body;
                VERIFY(k4abt_frame_get_body_skeleton(bodyFrame, i, &body.skeleton), "Get skeleton from body frame failed!");
                body.id = k4abt_frame_get_body_id(bodyFrame, i);

                Color color = g_bodyColors[body.id % g_bodyColors.size()];
                color.a = i == JumpEvaluationBodyIndex ? 0.8f : 0.1f;

                window3d.AddBody(body, color);
            }         
            k4a_capture_release(originalCapture);
            k4a_image_release(depthImage);
            k4abt_frame_release(bodyFrame);
        }
        window3d.Render();
        if (stream && !streamConnected) {
            Stream();
            streamConnected = true;
        }

    }

    if (!quit) {
        CreateRenderWindow(m_window3dReplay, "Replay", m_listOfBodyPositions[0], 2);

        milliseconds duration = milliseconds::zero();
        milliseconds expectedFrameDuration = milliseconds(33); // 30 fps (1000millisec/30 fps = ca 33 milliseconds pr frame)
        size_t currentReplayIndex = 0;
        m_reviewWindowIsRunning = true;
        while (m_reviewWindowIsRunning)
        {
            auto start = high_resolution_clock::now();
            if (duration > expectedFrameDuration)
            {
                currentReplayIndex = (currentReplayIndex + 1) % m_listOfBodyPositions.size();
                auto currentBody = m_listOfBodyPositions[currentReplayIndex];

                /*// Try to skip one frame if we detected a flip
                if (currentBody.skeleton.joints[K4ABT_JOINT_ANKLE_LEFT].position.xyz.x <=
                    currentBody.skeleton.joints[K4ABT_JOINT_ANKLE_RIGHT].position.xyz.x)
                {
                    currentReplayIndex = (currentReplayIndex + 1) % m_listOfBodyPositions.size();
                }*/

                m_window3dReplay.CleanJointsAndBones();
                m_window3dReplay.AddBody(m_listOfBodyPositions[currentReplayIndex], g_bodyColors[0]);
                duration = milliseconds::zero();
            }

            m_window3dReplay.Render();

            duration += duration_cast<milliseconds>(high_resolution_clock::now() - start);
        }
    }

    window3d.Delete();
    m_window3dReplay.Delete();
    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);

    k4a_device_stop_cameras(device);
    k4a_device_close(device);

    if (saveRecord) {
        printf("number of frames to be saved: %d\n", m_listOfBodyPositions.size());

        char str[200];
        FILE* fp;
        

        std::string newPath;
        std::string newPathCSV;

        bool validFileName = false;

        while (!validFileName) {
            printf("\n Enter a name for the file: ");
            scanf("%s", str);
            newPath = pathString + str + ".txt";
            newPathCSV = pathString + str + ".csv";
            if (fp = fopen(newPath.c_str(), "r")) {
                fclose(fp);
                printf("\n File name exits! choose another name\n");                
            }
            else {
                printf("\n valid File name, saving...");
                validFileName = true;
            }
                   
        }


            std::vector<std::pair<std::string, std::vector<float>>> vals;
                  
        //open the file for writing
        fp = fopen(newPath.c_str(), "w");
          
            //save all vector pos
             
            for (int z = 0; z < 32; z++) {
                std::string posx = "posX" + std::to_string(z);;
                std::string posy = "posY" + std::to_string(z);
                std::string posz = "posZ" + std::to_string(z);
                std::string posvel = "posVel" + std::to_string(z);
                std::string rotx = "rotX" + std::to_string(z);
                std::string roty = "rotY" + std::to_string(z);
                std::string rotz = "rotZ" + std::to_string(z);
                std::string rotvel = "rotVel" + std::to_string(z);
                std::string confi = "confidence" + std::to_string(z);

                vals.push_back({ posx, std::vector<float>() });
                vals.push_back({ posy,std::vector<float>() });
                vals.push_back({ posz,std::vector<float>() });
                vals.push_back({ posvel,std::vector<float>() });

                vals.push_back({ rotx,std::vector<float>() });
                vals.push_back({ roty,std::vector<float>() });
                vals.push_back({ rotz,std::vector<float>() });
                vals.push_back({ rotvel,std::vector<float>() });

                vals.push_back({ confi,std::vector<float>() });
            }




        for (int i = 0; i < m_listOfBodyPositions.size(); i++) {
           
            //timestamp in the file
            fprintf(fp, "%f,", m_framesTimestampInUsec[i]);

            for (int x = 0; x < (sizeof(m_listOfBodyPositions[0].skeleton.joints) / sizeof(m_listOfBodyPositions[0].skeleton.joints[0])); x++) {
                           
                //vector pos
                fprintf(fp, "%f,", m_listOfBodyPositions[i].skeleton.joints[x].position.v[0]); //x
                fprintf(fp, "%f,", m_listOfBodyPositions[i].skeleton.joints[x].position.v[1]); //y
                fprintf(fp, "%f,", m_listOfBodyPositions[i].skeleton.joints[x].position.v[2]); //z

                vals[x*9].second.push_back( m_listOfBodyPositions[i].skeleton.joints[x].position.v[0]);
                vals[x * 9 +1].second.push_back(m_listOfBodyPositions[i].skeleton.joints[x].position.v[1]);
                vals[x * 9 + 2].second.push_back(m_listOfBodyPositions[i].skeleton.joints[x].position.v[2]);
                if (i > 0) {
                    vals[x * 9 + 3].second.push_back(CalcVelGivenTwoPointsAndTime(m_listOfBodyPositions[i - 1].skeleton.joints[x].position.v[0], m_listOfBodyPositions[i - 1].skeleton.joints[x].position.v[1], m_listOfBodyPositions[i - 1].skeleton.joints[x].position.v[2], m_framesTimestampInUsec[i-1], m_listOfBodyPositions[i].skeleton.joints[x].position.v[0], m_listOfBodyPositions[i].skeleton.joints[x].position.v[1], m_listOfBodyPositions[i].skeleton.joints[x].position.v[2], m_framesTimestampInUsec[i]));
                    printf("x1 :%f, y1: %f, z1: %f, time1: %f, x2: %f, y2: %f, z2: %f,time2: %f \n", m_listOfBodyPositions[i - 1].skeleton.joints[x].position.v[0], m_listOfBodyPositions[i - 1].skeleton.joints[x].position.v[1], m_listOfBodyPositions[i - 1].skeleton.joints[x].position.v[2], m_framesTimestampInUsec[i-1], m_listOfBodyPositions[i].skeleton.joints[x].position.v[0], m_listOfBodyPositions[i].skeleton.joints[x].position.v[1], m_listOfBodyPositions[i].skeleton.joints[x].position.v[2], m_framesTimestampInUsec[i]);
                }
                else 
                    vals[x * 9 + 3].second.push_back(0.0);

                //Orientation
                fprintf(fp, "%f,", m_listOfBodyPositions[i].skeleton.joints[x].orientation.v[0]); //x
                fprintf(fp, "%f,", m_listOfBodyPositions[i].skeleton.joints[x].orientation.v[1]); //y
                fprintf(fp, "%f,", m_listOfBodyPositions[i].skeleton.joints[x].orientation.v[2]); //z


                vals[x * 9 + 4].second.push_back(m_listOfBodyPositions[i].skeleton.joints[x].orientation.v[0]);
                vals[x * 9 + 5].second.push_back(m_listOfBodyPositions[i].skeleton.joints[x].orientation.v[1]);
                vals[x * 9 + 6].second.push_back(m_listOfBodyPositions[i].skeleton.joints[x].orientation.v[2]);
                vals[x * 9 + 7].second.push_back(0.0);




                //confidence of joint
                fprintf(fp, "%d,", m_listOfBodyPositions[i].skeleton.joints[x].confidence_level); // cofidence [0-4] 0 lowest, 4 highest               
                vals[x * 9 + 8].second.push_back(m_listOfBodyPositions[i].skeleton.joints[x].confidence_level);
            }   
        }
        vals.push_back({ "TimeStamp", m_framesTimestampInUsec });

        
        // close the file
        fclose(fp);
        write_csv(newPathCSV, vals);
    }
    return 0;
}


