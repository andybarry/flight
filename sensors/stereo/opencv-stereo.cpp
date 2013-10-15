/**
 * Program that grabs stereo pairs from Point Grey
 * Firefly MV USB cameras, perfroms the single-disparity
 * stereo algorithm on them, and publishes the results
 * to LCM.
 *
 * Written by Andrew Barry <abarry@csail.mit.edu>, 2013
 *
 */

#include <cv.h>
#include <highgui.h>
#include "opencv2/legacy/legacy.hpp"
#include <sys/time.h>

#include "opencv2/opencv.hpp"

#include <GL/gl.h>
#include <lcm/lcm.h>
#include <bot_lcmgl_client/lcmgl.h>
#include "../../LCM/lcmt_stereo.h"
#include "../../LCM/lcmt_stereo_control.h"

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>


//#include <libusb.h> // for USB reset

#include "barrymoore.hpp"

using namespace std;
using namespace cv;

extern "C"
{
    #include <stdio.h>
    #include <stdint.h>
    #include <stdlib.h>
    #include <inttypes.h>

    #include <dc1394/dc1394.h>

    #include "camera.h"
    #include "utils.h"
}

#define MAX_HIT_POINTS 320*240/25 // this is the most hits we can get with our setup TODO: fixme for the correct framesize

//#define RINGBUFFER_SIZE (120*80) // number of seconds to allocate for recording * framerate
#define RINGBUFFER_SIZE (120*50) // number of seconds to allocate for recording * framerate

//#define POINT_GREY_VENDOR_ID 0x1E10
//#define POINT_GREY_PRODUCT_ID 0x2000

#define BRIGHTNESS_VALUE 78
#define EXPOSURE_VALUE 128

#define USE_IMAGE 0 // set to 1 to use left.jpg and right.jpg as test images

bool show_display; // set to true to show opencv images on screen

// allocate a huge array for a ringbuffer
Mat ringbufferL[RINGBUFFER_SIZE];
Mat ringbufferR[RINGBUFFER_SIZE];


// global for where we are drawing a line on the image
int lineLeftImgPosition = -1;
int lineLeftImgPositionY = -1;

// lcm subscription to the control channel
lcmt_stereo_control_subscription_t *stereo_control_sub;

struct RemapState
{
    Mat inputImage;
    Mat outputImage;
    Mat map1;
    Mat map2;
    int flags;
};

Mat GetFrameFormat7(dc1394camera_t *camera);

void MatchBrightnessSettings(dc1394camera_t *camera1, dc1394camera_t *camera2);
bool ResetPointGreyCameras();

int64_t getTimestampNow();
void WriteVideo();

void onMouse( int event, int x, int y, int, void* );
void DrawLines(Mat leftImg, Mat rightImg, Mat stereoImg, int lineX, int lineY, int disparity);

int numFrames = 0;
int recNumFrames = 0;
bool recordingOn = true;

dc1394_t        *d;
dc1394camera_t  *camera;

dc1394_t        *d2;
dc1394camera_t  *camera2;

//cv::vector<Point3f> *localHitPoints;

static void usage(void)
{
    fprintf(stderr, "usage: ./opencv-stereo stereo-control-channel [options]\n");
    fprintf(stderr, "\tstereo-control-channel: lcm channel to receive start and stop commands on\n");
    fprintf(stderr, "\t Options:\n");
    fprintf(stderr, "\t-v: show displays for debugging (substantially slows down processing)\n\n");
    exit(0);
}

void StopCapture()
{
    
    dc1394_video_set_transmission(camera, DC1394_OFF);
    dc1394_capture_stop(camera);
    dc1394_camera_free(camera);
    
    dc1394_video_set_transmission(camera2, DC1394_OFF);
    dc1394_capture_stop(camera2);
    dc1394_camera_free(camera2);
    
    dc1394_free (d);
    dc1394_free (d2);
    
    
}

/**
 * Cleanly handles and exit from a command-line
 * ctrl-c signal.
 *
 * @param s
 *
 */
void control_c_handler(int s)
{
    cout << endl << "exiting via ctrl-c" << endl
        << "\tpress ctrl+\\ to quit while writing video." << endl;
  
    StopCapture();
    WriteVideo();
    
    
    //delete[] localHitPoints;
    
    exit(1);

}



void lcm_stereo_control_handler(const lcm_recv_buf_t *rbuf, const char* channel, const lcmt_stereo_control *msg, void *user)
{
    /*
     * The way this works is that as soon as the program starts, stereo is always running.  The only thing that causes
     * the stereo to stop running is if stereoOn is set to false AND recOn is set to false.  The we write
     * the video we have to disk.
     * 
     * If recOn is false but stereoOn is true, we pause the recording
     * 
     */
     
    // got a control message, so parse it and figure out what we should do
    if (msg->stereoOn == false && msg->recOn == false)
    {
        // shut everything down and write the video out
        StopCapture();
        WriteVideo();
        
    } else if (msg->stereoOn == true && msg->recOn == false)
    {
        // pause recording if we were recording
        cout << "Recording stopped." << endl;
        recordingOn = false;
        
    } else if (msg->stereoOn == true && msg->recOn == true)
    {  
        cout << "(Re)starting recording." << endl;
        recordingOn = true;
        recNumFrames = 0;
    }
    {
        // got an unknown command
        fprintf(stderr, "WARNING: Unknown stereo_control command: stereoOn: %d, recOn: %d", int(msg->stereoOn), int(msg->recOn));
    }
    
}

void WriteVideo()
{
    printf("Writing video...\n");
    
    // get the date and time for the filename
    time_t rawtime;
    struct tm * timeinfo;
    char buffer [80];

    time ( &rawtime );
    timeinfo = localtime ( &rawtime );

    strftime (buffer,80,"%Y-%m-%d-%H-%M-%S",timeinfo);


    int endI, firstFrame = 0;
    if (recNumFrames < RINGBUFFER_SIZE)
    {
        endI = recNumFrames;
    } else {
        // our buffer is smaller than the full movie.
        // figure out where in the ringbuffer we are
        firstFrame = recNumFrames%RINGBUFFER_SIZE+1;
        if (firstFrame > RINGBUFFER_SIZE)
        {
            firstFrame = 0;
        }
        
        endI = RINGBUFFER_SIZE;
        
        printf("\nWARNING: buffer size exceeded by %d frames, which have been dropped.\n\n", recNumFrames-RINGBUFFER_SIZE);
        
    }
    
    VideoWriter recordL("videoL-skip-" + std::to_string(firstFrame) + "-" + string(buffer) +
        ".avi", CV_FOURCC('P','I','M','1'), 110, ringbufferL[0].size(), false);
    if( !recordL.isOpened() ) {
        printf("VideoWriter failed to open!\n");
    }
    VideoWriter recordR("videoR-skip-" + std::to_string(firstFrame) + "-" + string(buffer) +
        ".avi", CV_FOURCC('P','I','M','1'), 110, ringbufferR[0].size(), false);
    if( !recordR.isOpened() ) {
        printf("VideoWriter failed to open!\n");
    }
    
    // write the video
    for (int i=0; i<endI; i++)
    {
        recordL << ringbufferL[(i+firstFrame)%RINGBUFFER_SIZE];
        recordR << ringbufferR[(i+firstFrame)%RINGBUFFER_SIZE];
        
        printf("\rWriting video: (%.1f%%) -- %d/%d frames", (float)(i+1)/endI*100, i+1, endI);
        fflush(stdout);
    }
    printf("\ndone.\n");
}

void NonBlockingLcm(lcm_t *lcm)
{
    // setup an lcm function that won't block when we read it
    int lcm_fd = lcm_get_fileno(lcm);
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(lcm_fd, &fds);
    
    // wait a limited amount of time for an incoming message
    struct timeval timeout = {
        0,  // seconds
        1   // microseconds
    };

    
    int status = select(lcm_fd + 1, &fds, 0, 0, &timeout);

    if(0 == status) {
        // no messages
        //do nothing
        
    } else if(FD_ISSET(lcm_fd, &fds)) {
        // LCM has events ready to be processed.
        lcm_handle(lcm);
    }

}

int main(int argc, char *argv[])
{
    // get input arguments
    
    show_display = false;
    if (argc < 2) {
        cerr << "Did not provide at least 1 flag." << endl;
        exit(1);
    }
    
    char *stereo_control_channel_str;
    
    stereo_control_channel_str = argv[1];
    
    if (argc == 3)
    {
        // TODO: actually handle arguments instead of this hack
        show_display = true;
    }
    
    dc1394error_t   err;
    uint64         guid = 0x00b09d0100a01a9a; // camera 1
    
    unsigned long elapsed;
    
    // ----- cam 2 -----
    
    dc1394error_t   err2;
    uint64         guid2 = 0x00b09d0100a01ac5; // camera2
    
    // --- setup control-c handling ---
    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = control_c_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    // --- end ctrl-c handling code ---
    
    // tell opencv to use only one core so that we can manage our
    // own threading without a fight
    setNumThreads(1);
    
    #if !USE_IMAGE
        d = dc1394_new ();
        if (!d)
            cerr << "Could not create dc1394 context" << endl;
            
        d2 = dc1394_new ();
        if (!d2)
            cerr << "Could not create dc1394 context for camera 2" << endl;

        camera = dc1394_camera_new (d, guid);
        if (!camera)
        {
            cerr << "Could not create dc1394 camera... quitting." << endl;
            exit(1);
            #if 0
            if (!ResetPointGreyCameras())
            {
                // failed
                cerr << "Failed to reset USB cameras.  Quitting." << endl;
                exit(1);
            }
            #endif
        }
        
        camera2 = dc1394_camera_new (d2, guid2);
        if (!camera2)
            cerr << "Could not create dc1394 camera for camera 2" << endl;
        // reset the bus
        dc1394_reset_bus(camera);
        dc1394_reset_bus(camera2);
        
        // setup
        err = setup_gray_capture(camera, DC1394_VIDEO_MODE_FORMAT7_1);
        DC1394_ERR_CLN_RTN(err, cleanup_and_exit(camera), "Could not setup camera");

        err2 = setup_gray_capture(camera2, DC1394_VIDEO_MODE_FORMAT7_1);
        DC1394_ERR_CLN_RTN(err2, cleanup_and_exit(camera2), "Could not setup camera number 2");
        
        #if 0
        // enable auto-exposure
        // turn on the auto exposure feature
        err = dc1394_feature_set_power(camera, DC1394_FEATURE_EXPOSURE, DC1394_ON);
        DC1394_ERR_RTN(err,"Could not turn on the exposure feature");
        
        err = dc1394_feature_set_mode(camera, DC1394_FEATURE_EXPOSURE, DC1394_FEATURE_MODE_ONE_PUSH_AUTO);
        DC1394_ERR_RTN(err,"Could not turn on Auto-exposure");
        
        // enable auto-exposure
        // turn on the auto exposure feature
        err = dc1394_feature_set_power(camera2, DC1394_FEATURE_EXPOSURE, DC1394_ON);
        DC1394_ERR_RTN(err,"Could not turn on the exposure feature for cam2");
        
        err = dc1394_feature_set_mode(camera2, DC1394_FEATURE_EXPOSURE, DC1394_FEATURE_MODE_ONE_PUSH_AUTO);
        DC1394_ERR_RTN(err,"Could not turn on Auto-exposure for cam2");
        #endif
        
        // enable camera
        err = dc1394_video_set_transmission(camera, DC1394_ON);
        DC1394_ERR_CLN_RTN(err, cleanup_and_exit(camera), "Could not start camera iso transmission");
        err2 = dc1394_video_set_transmission(camera2, DC1394_ON);
        DC1394_ERR_CLN_RTN(err2, cleanup_and_exit(camera2), "Could not start camera iso transmission for camera number 2");
        
        dc1394_feature_set_absolute_control(camera, DC1394_FEATURE_GAIN, DC1394_OFF);
        dc1394_feature_set_absolute_control(camera2, DC1394_FEATURE_GAIN, DC1394_OFF);
        dc1394_feature_set_absolute_control(camera, DC1394_FEATURE_SHUTTER, DC1394_OFF);
        dc1394_feature_set_absolute_control(camera2, DC1394_FEATURE_SHUTTER, DC1394_OFF);
    #endif
	
    //#if SHOW_DISPLAY
    if (show_display)
    {
        namedWindow("Input", CV_WINDOW_AUTOSIZE);
        namedWindow("Input2", CV_WINDOW_AUTOSIZE);
        namedWindow("Depth", CV_WINDOW_AUTOSIZE);
        namedWindow("Stereo", CV_WINDOW_AUTOSIZE);
        
        setMouseCallback("Input", onMouse); // for drawing disparity lines
        setMouseCallback("Stereo", onMouse); // for drawing disparity lines
        
        cvMoveWindow("Input", 100, 100);
        cvMoveWindow("Stereo", 100, 370);
        cvMoveWindow("Input2", 500, 100);
        cvMoveWindow("Depth", 500, 370);
    }
    //#endif
    
    // load calibration

    Mat qMat, mx1Mat, my1Mat, mx2Mat, my2Mat;

    CvMat *Q = (CvMat *)cvLoad("calib/Q.xml",NULL,NULL,NULL);
    CvMat *mx1 = (CvMat *)cvLoad("calib/mx1.xml",NULL,NULL,NULL);
    CvMat *my1 = (CvMat *)cvLoad("calib/my1.xml",NULL,NULL,NULL);
    CvMat *mx2 = (CvMat *)cvLoad("calib/mx2.xml",NULL,NULL,NULL);
    CvMat *my2 = (CvMat *)cvLoad("calib/my2.xml",NULL,NULL,NULL);
    
    qMat = Mat(Q, true);
    mx1Mat = Mat(mx1,true);
    my1Mat = Mat(my1,true);
    mx2Mat = Mat(mx2,true);
    my2Mat = Mat(my2,true);
    
    Mat mx1fp, empty1, mx2fp, empty2;
    
    // this will convert to a fixed-point notation
    convertMaps(mx1Mat, my1Mat, mx1fp, empty1, CV_16SC2, true);
    convertMaps(mx2Mat, my2Mat, mx2fp, empty2, CV_16SC2, true);

    // start up LCM
    lcm_t * lcm;
    lcm = lcm_create("udpm://239.255.76.67:7667?ttl=1");
    bot_lcmgl_t* lcmgl = bot_lcmgl_init(lcm, "lcmgl-stereo");
    
    // subscribe to the stereo control channel
    stereo_control_sub = lcmt_stereo_control_subscribe(lcm, stereo_control_channel_str, &lcm_stereo_control_handler, NULL);
    
    Mat imgDisp;
    Mat imgDisp2;
    
    // initilize default parameters
    BarryMooreState state;
    
    state.disparity = -44;
    state.sobelLimit = 180; //130
    state.blockSize = 5;
    state.sadThreshold = 84;//79;
    state.sobelAdd = 0;
    
    state.mapxL = mx1fp;
    state.mapxR = mx2fp;
    state.Q = qMat;
    state.show_display = show_display;
    
    Mat matL, matR;
    bool quit = false;
    Mat depthMap;
    
    #if !USE_IMAGE
    // allocate a huge buffer for video frames
    printf("Allocating ringbuffer data...\n");
    matL = GetFrameFormat7(camera);
    for (int i=0; i<RINGBUFFER_SIZE; i++)
    {
        ringbufferL[i].create(matL.size(), matL.type());
        ringbufferR[i].create(matL.size(), matL.type());
    }
    printf("done.\n");
    #endif
    //localHitPoints = new cv::vector<Point3f>[NUM_THREADS];
    //state.localHitPoints = localHitPoints;
    //
    //for (int i=0; i< NUM_THREADS; i++)
    //{
    //    localHitPoints[i].resize(MAX_HIT_POINTS);
    //}
    
    
    #if USE_IMAGE
        matL = imread("left.jpg", 0);
        matR = imread("right.jpg", 0);
    #endif
    
    // start the framerate clock
    struct timeval start, now;
    gettimeofday( &start, NULL );
    
    while (quit == false) {
    
        // get the frames from the camera
        #if !USE_IMAGE
        
            // we would like to match brightness every frame
            // but that would really hurt our framerate
            // match brightness every 10 frames instead
            if (numFrames % 10 == 0)
            {
                MatchBrightnessSettings(camera, camera2);
            }
        
            matL = GetFrameFormat7(camera);
            matR = GetFrameFormat7(camera2);
            
            // record video
            if (recordingOn == true)
            {
                ringbufferL[recNumFrames%RINGBUFFER_SIZE] = matL;
                ringbufferR[recNumFrames%RINGBUFFER_SIZE] = matR;
                
                recNumFrames ++;
            }
        #endif
        
        
        
        Mat matDisp, remapL, remapR;
        
        //#if SHOW_DISPLAY
        if (show_display)
        {
            // we remap again here because we're just in display
            Mat remapLtemp(matL.rows, matL.cols, matL.depth());
            Mat remapRtemp(matR.rows, matR.cols, matR.depth());
            
            remapL = remapLtemp;
            remapR = remapRtemp;
            
            if (numFrames == 0)
            {
                depthMap = Mat::zeros(matL.rows, matL.cols, CV_8UC1);
            }
            
            remap(matL, remapL, mx1fp, Mat(), INTER_NEAREST);
            remap(matR, remapR, mx2fp, Mat(), INTER_NEAREST);
            
            remapL.copyTo(matDisp);
        }
        //#endif
        
        cv::vector<Point3f> pointVector3d;
        cv::vector<uchar> pointColors;
        cv::vector<Point3i> pointVector2d; // for display
        
        
        StereoBarryMoore(matL, matR, &pointVector3d, &pointColors, &pointVector2d, state);
        
        lcmt_stereo msg;
        msg.timestamp = getTimestampNow();
        msg.number_of_points = (int)pointVector3d.size();
        
        float x[msg.number_of_points];
        float y[msg.number_of_points];
        float z[msg.number_of_points];
        uchar grey[msg.number_of_points];
        
        for (unsigned int i=0;i<pointVector3d.size();i++)
        {
            x[i] = pointVector3d[i].x;
            y[i] = pointVector3d[i].y;
            z[i] = pointVector3d[i].z;
            grey[i] = pointColors[i];
            
        }
        
        msg.x = x;
        msg.y = y;
        msg.z = z;
        msg.grey = grey;
        msg.frame_number = numFrames;
        
        lcmt_stereo_publish(lcm, "stereo", &msg);
        
        // publish points to the 3d map
        //PublishToLcm(lcmgl, matDisp, Q, matR);
        
        // prep disparity image for display
        //normalize(matDisp, matDisp, 0, 256, NORM_MINMAX, CV_8S);
        
        // display images
        #if USE_LCMGL
            bot_lcmgl_push_matrix(lcmgl);
            bot_lcmgl_point_size(lcmgl, 10.5f);
            bot_lcmgl_begin(lcmgl, GL_POINTS);
            
            for (unsigned int i=0;i<pointVector3d.size();i++)
            {
                float x = pointVector3d[i].x;
                float y = pointVector3d[i].y;
                float z = pointVector3d[i].z;
                // publish to LCM
                
                #if 0
                //#if SHOW_DISPLAY
                if (show_display)
                {
                 
                     //cout << "(" << x << ", " << y << ", " << z << ")" << endl;
                    
                    // get the color of the pixel
                    int x2 = pointVector2d[i].x;
                    int y2 = pointVector2d[i].y;
                    double factor = 2.0;
                    //int delta = (state.blockSize-1)/2;
                    int delta = 20;
                   
                    for (int row=y2-delta;row<y2+delta;row++)
                    {
                        int rowdelta = row-y2*factor;
                        for (int col = x2-delta; col < x2+delta; col++)
                        {
                            int coldelta = col-x2*factor;
                        
                            uchar pxL = remapL.at<uchar>(row, col);
                            float gray = pxL/255.0;
                        
                            bot_lcmgl_color3f(lcmgl, gray, gray, gray);                    
                            bot_lcmgl_vertex3f(lcmgl, x+coldelta, -y-rowdelta, z);
                        }
                    }
                }
                //#endif
                
                #endif
            
                bot_lcmgl_vertex3f(lcmgl, z, x, -y);
            }
            
            bot_lcmgl_end(lcmgl);
            bot_lcmgl_pop_matrix(lcmgl);
        
         
         

            //if (numFrames%200 == 0)
            {
            bot_lcmgl_switch_buffer(lcmgl);
            }
        #endif //USE_LCMGL
        
        //#if SHOW_DISPLAY
        if (show_display)
        {
            // 1b. Convert image into a GL texture:
            int row_stride = remapL.cols; // 1*width
            int height = remapL.rows;
            int width = remapL.cols;
            int gray_texid = bot_lcmgl_texture2d(lcmgl, remapL.data, remapL.cols, remapL.rows, row_stride, BOT_LCMGL_LUMINANCE,
                                           BOT_LCMGL_UNSIGNED_BYTE,
                                           BOT_LCMGL_COMPRESS_NONE);
            bot_lcmgl_push_matrix(lcmgl);
            bot_lcmgl_color3f(lcmgl, 1, 1, 1);
            bot_lcmgl_translated(lcmgl, 0, 400, 0); // example offset
            bot_lcmgl_texture_draw_quad(lcmgl, gray_texid,
            0     , 0      , 0   ,
            0     , height , 0   ,
            -width , height , 0   ,   ///... where to put the image
            -width , 0      , 0);  

            bot_lcmgl_pop_matrix(lcmgl);
            bot_lcmgl_switch_buffer(lcmgl);
            
            for (unsigned int i=0;i<pointVector2d.size();i++)
            {
                int x2 = pointVector2d[i].x;
                int y2 = pointVector2d[i].y;
                int sad = pointVector2d[i].z;
                rectangle(matDisp, Point(x2,y2), Point(x2+state.blockSize, y2+state.blockSize), sad,  CV_FILLED);
                rectangle(matDisp, Point(x2+1,y2+1), Point(x2+state.blockSize-1, y2-1+state.blockSize), 255);
                
                int gray = 337.0 - state.disparity*41.0/6;
                
                // fill in the depth map at this point
                circle(depthMap, Point(x2,y2), 5, gray, -1);
            }
            
            // draw a line for the user to show disparity
            DrawLines(remapL, remapR, matDisp, lineLeftImgPosition, lineLeftImgPositionY, state.disparity);
            
            imshow("Input", remapL);
            imshow("Input2", remapR);
            imshow("Stereo", matDisp);
            //imshow("Depth", depthMap);
                
            char key = waitKey(1);
            
            if (key != 255)
            {
                cout << endl << key << endl;
            }
            
            switch (key)
            {
                case 'T':
                    state.disparity --;
                    break;
                case 'R':
                    state.disparity ++;
                    break;
                    
                case 'w':
                    state.sobelLimit += 10;
                    break;
                    
                case 's':
                    state.sobelLimit -= 10;
                    break;
                    
                case 'g':
                    state.blockSize ++;
                    break;
                    
                case 'b':
                    state.blockSize --;
                    break;
                    
                case 'y':
                    state.sadThreshold ++;
                    break;
                    
                case 'h':
                    state.sadThreshold --;
                    break;
                    
                case 'u':
                    state.sobelAdd += 10;
                    break;
                    
                case 'j':
                    state.sobelAdd -= 10;
                    break;
                
                case '5':
                    // to show SAD boxes
                    state.sobelLimit = 0;
                    state.sadThreshold = 255;
                    break;
                    
                case 'q':
                    quit = true;
                    break;
            }
            
            if (key != 255)
            {
                cout << "disparity = " << state.disparity << endl;
                cout << "sobelLimit = " << state.sobelLimit << endl;
                cout << "blockSize = " << state.blockSize << endl;
                cout << "sadThreshold = " << state.sadThreshold << endl;
                cout << "sobelAdd = " << state.sobelAdd << endl;
            }
        }
        //#endif
        
        numFrames ++;
        
        // check for new LCM messages
        NonBlockingLcm(lcm);
            
        // compute framerate
        gettimeofday( &now, NULL );
        
        elapsed = (now.tv_usec / 1000 + now.tv_sec * 1000) - 
        (start.tv_usec / 1000 + start.tv_sec * 1000);
        
        printf("\r%d frames (%lu ms) - %4.1f fps | %4.1f ms/frame", numFrames, elapsed, (float)numFrames/elapsed * 1000, elapsed/(float)numFrames);
        fflush(stdout);

        
    }

    printf("\n\n");
    
    destroyWindow("Input");
    destroyWindow("Input2");
    destroyWindow("Stereo");
    
    // stop data transmission
    err = dc1394_video_set_transmission(camera, DC1394_OFF);
    DC1394_ERR_CLN_RTN(err,cleanup_and_exit(camera),"Could not stop the camera");
    
    err2 = dc1394_video_set_transmission(camera2, DC1394_OFF);
    DC1394_ERR_CLN_RTN(err2,cleanup_and_exit(camera2),"Could not stop the camera 2");

    // close camera
    StopCapture();
    
    //delete[] localHitPoints;
    
    return 0;
}

/**
 * Gets a Format7 frame from a Firefly MV USB camera.
 * The frame will be CV_8UC1 and black and white.
 *
 * @param camera the camrea
 *
 * @retval frame as an OpenCV Mat
 */
Mat GetFrameFormat7(dc1394camera_t *camera)
{
    // get a frame
    dc1394error_t err;
    dc1394video_frame_t *frame;

    err = dc1394_capture_dequeue(camera, DC1394_CAPTURE_POLICY_WAIT, &frame);
    DC1394_WRN(err,"Could not capture a frame");
    
    // make a Mat of the right size and type that we attach the the existing data
    Mat matTmp = Mat(frame->size[1], frame->size[0], CV_8UC1, frame->image, frame->size[0]);
    
    // copy the data out of the ringbuffer so we can give the buffer back
    Mat matOut = matTmp.clone();
    
    // release the buffer
    err = dc1394_capture_enqueue(camera, frame);
    DC1394_WRN(err,"releasing buffer");
    
    return matOut;
}

int64_t getTimestampNow()
{
    struct timeval thisTime;
    gettimeofday(&thisTime, NULL);
    return (thisTime.tv_sec * 1000000.0) + (float)thisTime.tv_usec + 0.5;
}

void MatchBrightnessSettings(dc1394camera_t *camera1, dc1394camera_t *camera2)
{
    // we need to make sure the brightness settings
    // are identical on both cameras. to do this, we
    // read the settings off the left camera, and force
    // the right camera to do the exact same thing.
    // since we want auto-brightness, we do this every frame
    
    // in practice, it is really important to get this right
    // otherwise the pixel values will be totally different
    // and stereo will not work at all
    
    
    
    #if 0
    dc1394error_t err;

    //dc1394_feature_print(&features.feature[8], stdout);
    
    
    // set brightness
    dc1394_feature_set_value(camera2, features.feature[0].id, features.feature[0].value);
    
    // set exposure
    dc1394_feature_set_value(camera2, features.feature[1].id, features.feature[1].value);
    
    // set gamma
    dc1394_feature_set_value(camera2, features.feature[6].id, features.feature[6].value);
    
    // set shutter
    dc1394_feature_set_absolute_value(camera2, features.feature[7].id, features.feature[7].abs_value);
    #endif
    
    // set shutter
    uint32_t shutterVal;
    dc1394_feature_get_value(camera1, DC1394_FEATURE_SHUTTER, &shutterVal);
    dc1394_feature_set_value(camera2, DC1394_FEATURE_SHUTTER, shutterVal);
    
    
    
    // set gain
    uint32_t gainVal;
    dc1394_feature_get_value(camera1, DC1394_FEATURE_GAIN, &gainVal);
    
    dc1394_feature_set_value(camera2, DC1394_FEATURE_GAIN, gainVal);
    //DC1394_WRN(err,"Could not set gain");
    
    
    #if 0
    dc1394featureset_t features, features2;
    dc1394_feature_get_all(camera1, &features);
    
    // set framerate
    dc1394_feature_set_absolute_value(camera2, features.feature[15].id, features.feature[15].abs_value);
    
    dc1394_feature_get_all(camera2, &features2);
    
    
    
    cout << endl << dc1394_feature_get_string(features.feature[0].id) << ": " << features.feature[0].value << "/" << features2.feature[0].value << endl;

    
    // set exposure
    cout << endl << dc1394_feature_get_string(features.feature[1].id) << ": " << features.feature[1].value << "/" << features2.feature[1].value << endl;

    
    // set gamma
    cout << endl << dc1394_feature_get_string(features.feature[6].id) << ": " << features.feature[6].value << "/" << features2.feature[6].value << endl;

    
    // set shutter
    cout << endl << dc1394_feature_get_string(features.feature[7].id) << ": " << features.feature[7].value << "/" << features2.feature[7].value << endl;
    
    // set gain
   cout << endl << dc1394_feature_get_string(features.feature[8].id) << ": " << features.feature[8].value << "/" << features2.feature[8].value << endl;
    
    //cout << "---------------------------------------" << endl;
    //dc1394_feature_print_all(&features, stdout);
    //cout << "222222222222222222222222222222222222222222222" << endl;
    //dc1394_feature_print_all(&features2, stdout);
    //dc1394_feature_get_absolute_value(camera1, DC1394_FEATURE_BRIGHTNESS, &brightnessVal);
    
    //cout << endl << "brightness: " << brightnessVal << endl;
    #endif
}

/* setup a mouse callback so that the user can click on an image
 * and see where the disparity line is on the other image pair
 */
void onMouse( int event, int x, int y, int flags, void* )
{
    if( flags & CV_EVENT_FLAG_LBUTTON)
    {
        // paint a line on the image they clicked on
        lineLeftImgPosition = x;
        lineLeftImgPositionY = y;
    }
}

void DrawLines(Mat leftImg, Mat rightImg, Mat stereoImg, int lineX, int lineY, int disparity)
{
    int lineColor = 128;
    if (lineX >= 0)
    {
        // print out the values of the pixels where they clicked
        cout << endl << endl << "Left px: " << (int)leftImg.at<uchar>(lineY, lineX)
            << "\tRight px: " << (int)rightImg.at<uchar>(lineY, lineX + disparity)
            << endl;
            
        line(leftImg, Point(lineX, 0), Point(lineX, leftImg.rows), lineColor);
        line(stereoImg, Point(lineX, 0), Point(lineX, leftImg.rows), lineColor);
        line(rightImg, Point(lineX + disparity, 0), Point(lineX + disparity, rightImg.rows), lineColor);
        
        line(leftImg, Point(0, lineY), Point(leftImg.cols, lineY), lineColor);
        line(stereoImg, Point(0, lineY), Point(leftImg.cols, lineY), lineColor);
        line(rightImg, Point(0, lineY), Point(rightImg.cols, lineY), lineColor);
    }
}

# if 0
/**
 * Function that finds and resets all connected USB Point Grey cameras.
 *
 * @retval true on successful reset, false on failure.
 */
bool ResetPointGreyCameras()
{
    libusb_context *s_libucontext;
    
    // open the libusb context
    libusb_init(&s_libucontext);
    
    libusb_device **list;

    ssize_t ret;

    ret = libusb_get_device_list (s_libucontext, &list);

    if (ret == LIBUSB_ERROR_NO_MEM || ret < 1)
    {
        printf("Failed to find device in the libusb context.\n");
        libusb_free_device_list(list, 1);
        libusb_exit(s_libucontext);
        return false;
    }
    
    int j;
    int libusb_flag = 0;
    
    for (j=0; j<ret; j++)
    {
        libusb_device *this_device = list[j];
        struct libusb_device_descriptor desc;

        if (libusb_get_device_descriptor(this_device, &desc) == 0)
        {
            // before attempting to open the device and get it's description, look at the vendor and product IDs
            // this also means that we won't attempt to open a bunch of devices we probably don't have permissions to open
            // furthermore, if the user hasn't set up permissions correctly, libusb will print out an error about the permissions
            // giving a good hint at what to do
            
            cout << desc.idVendor << endl;
            
            if (desc.idVendor == POINT_GREY_VENDOR_ID && desc.idProduct == POINT_GREY_PRODUCT_ID)
            {
                cout << "found a hit" << endl;

                libusb_device_handle *udev;
                if (libusb_open(this_device, &udev) == 0)
                {
                    cout << "resetting device." << endl;
                    // reset this device
                    cout << libusb_reset_device(udev);
                    cout << "success." << endl;
                    //libusb_close(udev);
                    libusb_flag = 1;
                } else
                {
                    cout << "failed to open device" << endl;
                }
            }
        }
    }
    
    // cleanup libusb list
    libusb_free_device_list(list, 1);
    
    libusb_exit(s_libucontext);

    if (libusb_flag == 0)
    {
        printf("Failed to reset the USB cameras.\n");
        return false;
    }
    
    return true;
    
}
#endif
