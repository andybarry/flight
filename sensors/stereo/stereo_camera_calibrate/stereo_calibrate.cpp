#pragma warning( disable: 4996 )
/* *************** License:**************************
   Oct. 3, 2008
   Right to use this code in any way you want without warrenty, support or any guarentee of it working.

   BOOK: It would be nice if you cited it:
   Learning OpenCV: Computer Vision with the OpenCV Library
     by Gary Bradski and Adrian Kaehler
     Published by O'Reilly Media, October 3, 2008
 
   AVAILABLE AT: 
     http://www.amazon.com/Learning-OpenCV-Computer-Vision-Library/dp/0596516134
     Or: http://oreilly.com/catalog/9780596516130/
     ISBN-10: 0596516134 or: ISBN-13: 978-0596516130    

   OTHER OPENCV SITES:
   * The source code is on sourceforge at:
     http://sourceforge.net/projects/opencvlibrary/
   * The OpenCV wiki page (As of Oct 1, 2008 this is down for changing over servers, but should come back):
     http://opencvlibrary.sourceforge.net/
   * An active user group is at:
     http://tech.groups.yahoo.com/group/OpenCV/
   * The minutes of weekly OpenCV development meetings are at:
     http://pr.willowgarage.com/wiki/OpenCV
   ************************************************** */

/*
	Modified by Martin Peris Martorell (info@martinperis.com) in order to accept some configuration
	parameters and store all the calibration data as xml files.

*/

#include "cv.h"
#include "cxmisc.h"
#include "highgui.h"
#include "cvaux.h"
#include <vector>
#include <string>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include <iostream>

#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

// global flags for camera calibration
bool calibrateOnlyLeft = false;
bool calibrateOnlyRight = false;

//
// Given a list of chessboard images, the number of corners (nx, ny)
// on the chessboards, and a flag: useCalibrated for calibrated (0) or
// uncalibrated (1: use cvStereoCalibrate(), 2: compute fundamental
// matrix separately) stereo. Calibrate the cameras and display the
// rectified results along with the computed disparity images.
//
static void
StereoCalib(const char* imageList, int nx, int ny, int useUncalibrated, float _squareSize)
{
    int displayCorners = 1;
    int showUndistorted = 1;
    bool isVerticalStereo = false;//OpenCV can handle left-right
                                      //or up-down camera arrangements
    const int maxScale = 1;
    const float squareSize = _squareSize; //Chessboard square size in cm
    FILE* f = fopen(imageList, "rt");
    int i, j, lr, nframes, n = nx*ny, N = 0;
    vector<string> imageNames[2];
    vector<CvPoint3D32f> objectPoints;
    vector<CvPoint2D32f> points[2];
    vector<int> npoints;
    vector<uchar> active[2];
    vector<CvPoint2D32f> temp(n);
    CvSize imageSize = {0,0};
    // ARRAY AND VECTOR STORAGE:
    double M1[3][3], M2[3][3], D1[8], D2[8];
    double R[3][3], T[3], E[3][3], F[3][3];
    double Q[4][4];
    
    
    CvMat _M1 = cvMat(3, 3, CV_64F, M1 );
    CvMat _M2 = cvMat(3, 3, CV_64F, M2 );
    CvMat _D1 = cvMat(1, 8, CV_64F, D1 );
    CvMat _D2 = cvMat(1, 8, CV_64F, D2 );

    bool full_stereo_calibration = false;
    
    if (calibrateOnlyLeft == false && calibrateOnlyRight == false)
    {
        // doing full stereo calibration, so require loading camera paramters
        full_stereo_calibration = true;
        
        printf("Doing a full stereo calibration.  Attemping to load camera paramters: M1-single.xml, D1-single.xml, M2-single.xml, and D2-single.xml...\n");
        
        CvMat *M1p = (CvMat *)cvLoad("M1-single.xml",NULL,NULL,NULL);
        if (M1p == NULL)
        {
            fprintf(stderr, "Error: Failed to load M1-single.xml\n");
            exit(-1);
        } else {
            _M1 = *M1p;
        }
        
        CvMat *D1p = (CvMat *)cvLoad("D1-single.xml",NULL,NULL,NULL);
        if (D1p == NULL)
        {
            fprintf(stderr, "Error: Failed to load D1-single.xml\n");
            exit(-1);
        } else {
            _D1 = *D1p;
        }
        
        CvMat *M2p = (CvMat *)cvLoad("M2-single.xml",NULL,NULL,NULL);
        if (M2p == NULL)
        {
            fprintf(stderr, "Error: Failed to load M2-single.xml\n");
            exit(-1);
        } else {
            _M2 = *M2p;
        }
        
        CvMat *D2p = (CvMat *)cvLoad("D2-single.xml",NULL,NULL,NULL);
        if (D2p == NULL)
        {
            fprintf(stderr, "Error: Failed to load D2-single.xml\n");
            exit(-1);
        } else {
            _D2 = *D2p;
        }
        
        printf("XML files loaded successfully.\n");
    }
    
    
    
    CvMat _R = cvMat(3, 3, CV_64F, R );
    CvMat _T = cvMat(3, 1, CV_64F, T );
    CvMat _E = cvMat(3, 3, CV_64F, E );
    CvMat _F = cvMat(3, 3, CV_64F, F );
    CvMat _Q = cvMat(4,4, CV_64F, Q);
    
    
    if( displayCorners )
        cvNamedWindow( "corners", 1 );
// READ IN THE LIST OF CHESSBOARDS:
    if( !f )
    {
        fprintf(stderr, "can not open file %s\n", imageList );
        return;
    }
    
    for(i=0;;i++)
    {
        char buf[1024];
        int count = 0, result=0;
        
        if (calibrateOnlyLeft == true)
        {
            lr = 0;
        } else if (calibrateOnlyRight == true)
        {
            lr = 1;
        } else {
            lr = i % 2;
        }
        vector<CvPoint2D32f>& pts = points[lr];
        if( !fgets( buf, sizeof(buf)-3, f ))
            break;
        size_t len = strlen(buf);
        while( len > 0 && isspace(buf[len-1]))
            buf[--len] = '\0';
        if( buf[0] == '#')
            continue;
        IplImage* img = cvLoadImage( buf, 0 );
        if( !img )
            break;
        imageSize = cvGetSize(img);
        imageNames[lr].push_back(buf);
        
        
    //FIND CHESSBOARDS AND CORNERS THEREIN:
        for( int s = 1; s <= maxScale; s++ )
        {
            IplImage* timg = img;
            if( s > 1 )
            {
                timg = cvCreateImage(cvSize(img->width*s,img->height*s),
                    img->depth, img->nChannels );
                cvResize( img, timg, CV_INTER_CUBIC );
            }
            result = cvFindChessboardCorners( timg, cvSize(nx, ny),
                &temp[0], &count,
                CV_CALIB_CB_ADAPTIVE_THRESH |
                CV_CALIB_CB_NORMALIZE_IMAGE);
            if( timg != img )
                cvReleaseImage( &timg );
            if( result || s == maxScale )
                for( j = 0; j < count; j++ )
            {
                temp[j].x /= s;
                temp[j].y /= s;
            }
            if( result )
                break;
        }
        if( displayCorners )
        {
            printf("%s\n", buf);
            IplImage* cimg = cvCreateImage( imageSize, 8, 3 );
            cvCvtColor( img, cimg, CV_GRAY2BGR );
            cvDrawChessboardCorners( cimg, cvSize(nx, ny), &temp[0],
                count, result );
            cvShowImage( "corners", cimg );
            cvReleaseImage( &cimg );
            if( cvWaitKey(0) == 27 ) //Allow ESC to quit
                exit(-1);
        }
        else
            putchar('.');
        N = pts.size();
        pts.resize(N + n, cvPoint2D32f(0,0));
        active[lr].push_back((uchar)result);
    //assert( result != 0 );
        if( result )
        {
         //Calibration will suffer without subpixel interpolation
            cvFindCornerSubPix( img, &temp[0], count,
                cvSize(11, 11), cvSize(-1,-1),
                cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,
                30, 0.01) );
            copy( temp.begin(), temp.end(), pts.begin() + N );
        }
        cvReleaseImage( &img );
        
    }
    fclose(f);
    printf("\n");
// HARVEST CHESSBOARD 3D OBJECT POINT LIST:
    if (calibrateOnlyRight == true)
    {
        nframes = active[1].size();
    } else {
        nframes = active[0].size();//Number of good chessboads found
    }
    printf("\nusing %d chessboards.\n", nframes);
    objectPoints.resize(nframes*n);
    for( i = 0; i < ny; i++ )
        for( j = 0; j < nx; j++ )
        objectPoints[i*nx + j] = cvPoint3D32f(i*squareSize, j*squareSize, 0);
    for( i = 1; i < nframes; i++ )
        copy( objectPoints.begin(), objectPoints.begin() + n,
        objectPoints.begin() + i*n );
    npoints.resize(nframes,n);
    N = nframes*n;
    CvMat _objectPoints = cvMat(1, N, CV_32FC3, &objectPoints[0] );
    CvMat _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
    CvMat _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
    CvMat _npoints = cvMat(1, npoints.size(), CV_32S, &npoints[0] );
    //cvSetIdentity(&_M1);
    //cvSetIdentity(&_M2);
    //cvZero(&_D1);
    //cvZero(&_D2);
    
    cvNamedWindow( "rect", 1 );
    
    
// CALIBRATE THE STEREO CAMERAS
    
    // check to see if we are doing a full stereo calibration or individual calibration
    if (calibrateOnlyLeft == true || calibrateOnlyRight == true)
    {
        if (calibrateOnlyLeft == true)
        {
            printf("Calibrating left camera...\n");
            
            cvCalibrateCamera2(&_objectPoints, &_imagePoints1, &_npoints, imageSize, &_M1, &_D1, NULL, NULL, CV_CALIB_RATIONAL_MODEL, cvTermCriteria( CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,30,DBL_EPSILON) );
            
            cvSave("M1-single.xml",&_M1);
            cvSave("D1-single.xml",&_D1);
            
            printf("Calibration saved to M1-single.xml and D1-single.xml\n");
            
            
        } else {
            printf("Calibrating right camera...\n");
            
            cvCalibrateCamera2(&_objectPoints, &_imagePoints2, &_npoints, imageSize, &_M2, &_D2, NULL, NULL, CV_CALIB_RATIONAL_MODEL, cvTermCriteria( CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,30,DBL_EPSILON) );
            
            cvSave("M2-single.xml",&_M2);
            cvSave("D2-single.xml",&_D2);
            
            printf("Calibration saved to M2-single.xml and D2-single.xml\n");
            
        }
        
        
        
    } else { // doing a full stereo calibration
    
    
        printf("Running stereo calibration ...\n");
        fflush(stdout);
        
         CvScalar scal;
        for(i=0;i<8;i++)
        {
            scal = cvGet1D(&_D1,i);
            printf("val:%f \n",scal.val[0]);
        }
        

        
        
        cvStereoCalibrate( &_objectPoints, &_imagePoints1,
            &_imagePoints2, &_npoints,
            &_M1, &_D1, &_M2, &_D2,
            imageSize, &_R, &_T, &_E, &_F,
            cvTermCriteria(CV_TERMCRIT_ITER+
            CV_TERMCRIT_EPS, 30, 1e-6),
            CV_CALIB_RATIONAL_MODEL | CV_CALIB_FIX_INTRINSIC);//|
//                CV_CALIB_FIX_ASPECT_RATIO );
        printf(" done\n");
        
         for(i=0;i<8;i++)
        {
            scal = cvGet1D(&_D1,i);
            printf("val:%f \n",scal.val[0]);
        }
    } // end for single or multiple camera calibration if
// CALIBRATION QUALITY CHECK
// because the output fundamental matrix implicitly
// includes all the output information,
// we can check the quality of calibration using the
// epipolar geometry constraint: m2^t*F*m1=0
    vector<CvPoint3D32f> lines[2];
    points[0].resize(N);
    points[1].resize(N);
    _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
    _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
    lines[0].resize(N);
    lines[1].resize(N);
    CvMat _L1 = cvMat(1, N, CV_32FC3, &lines[0][0]);
    CvMat _L2 = cvMat(1, N, CV_32FC3, &lines[1][0]);
//Always work in undistorted space
    cvUndistortPoints( &_imagePoints1, &_imagePoints1,
        &_M1, &_D1, 0, &_M1 );
    cvUndistortPoints( &_imagePoints2, &_imagePoints2,
        &_M2, &_D2, 0, &_M2 );
    cvComputeCorrespondEpilines( &_imagePoints1, 1, &_F, &_L1 );
    cvComputeCorrespondEpilines( &_imagePoints2, 2, &_F, &_L2 );
    double avgErr = 0;
    for( i = 0; i < N; i++ )
    {
        double err = fabs(points[0][i].x*lines[1][i].x +
            points[0][i].y*lines[1][i].y + lines[1][i].z)
            + fabs(points[1][i].x*lines[0][i].x +
            points[1][i].y*lines[0][i].y + lines[0][i].z);
        avgErr += err;
    }
    printf( "avg err = %g\n", avgErr/(nframes*n) );
//COMPUTE AND DISPLAY RECTIFICATION
    if( showUndistorted )
    {
        CvMat* mx1 = cvCreateMat( imageSize.height,
            imageSize.width, CV_32F );
        CvMat* my1 = cvCreateMat( imageSize.height,
            imageSize.width, CV_32F );
        CvMat* mx2 = cvCreateMat( imageSize.height,

            imageSize.width, CV_32F );
        CvMat* my2 = cvCreateMat( imageSize.height,
            imageSize.width, CV_32F );
        CvMat* img1r = cvCreateMat( imageSize.height,
            imageSize.width, CV_8U );
        CvMat* img2r = cvCreateMat( imageSize.height,
            imageSize.width, CV_8U );
        CvMat* disp = cvCreateMat( imageSize.height,
            imageSize.width, CV_16S );
        CvMat* vdisp = cvCreateMat( imageSize.height,
            imageSize.width, CV_8U );
        CvMat* pair;
        double R1[3][3], R2[3][3], P1[3][4], P2[3][4];
        CvMat _R1 = cvMat(3, 3, CV_64F, R1);
        CvMat _R2 = cvMat(3, 3, CV_64F, R2);
// IF BY CALIBRATED (BOUGUET'S METHOD)
        if( useUncalibrated == 0 )
        {
            CvMat _P1 = cvMat(3, 4, CV_64F, P1);
            CvMat _P2 = cvMat(3, 4, CV_64F, P2);
            cvStereoRectify( &_M1, &_M2, &_D1, &_D2, imageSize,
                &_R, &_T,
                &_R1, &_R2, &_P1, &_P2, &_Q,
                0/*CV_CALIB_ZERO_DISPARITY*/ );
            isVerticalStereo = fabs(P2[1][3]) > fabs(P2[0][3]);
    //Precompute maps for cvRemap()
            if (calibrateOnlyLeft == true || calibrateOnlyRight == true)
            {
                cvInitUndistortMap(&_M1,&_D1,mx1,my1);
                cvInitUndistortMap(&_M2,&_D2,mx2,my2);            
            } else {
                cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_P1,mx1,my1);
                cvInitUndistortRectifyMap(&_M2,&_D2,&_R2,&_P2,mx2,my2);
            }
            
            
            
    //Save parameters
    
            // don't resave if we're only calibrating one camera
            if (calibrateOnlyLeft == false && calibrateOnlyRight == false)
            {
                printf("Saving full stereo calibration...\n");
                cvSave("M1.xml",&_M1);
                cvSave("D1.xml",&_D1);
                cvSave("R1.xml",&_R1);
                cvSave("P1.xml",&_P1);
                cvSave("M2.xml",&_M2);
                cvSave("D2.xml",&_D2);
                cvSave("R2.xml",&_R2);
                cvSave("P2.xml",&_P2);
                cvSave("Q.xml",&_Q);
                cvSave("mx1.xml",mx1);
                cvSave("my1.xml",my1);
                cvSave("mx2.xml",mx2);
                cvSave("my2.xml",my2);
                printf("done.\n");
            }

        }
//OR ELSE HARTLEY'S METHOD
        else if( useUncalibrated == 1 || useUncalibrated == 2 )
     // use intrinsic parameters of each camera, but
     // compute the rectification transformation directly
     // from the fundamental matrix
        {
            double H1[3][3], H2[3][3], iM[3][3];
            CvMat _H1 = cvMat(3, 3, CV_64F, H1);
            CvMat _H2 = cvMat(3, 3, CV_64F, H2);
            CvMat _iM = cvMat(3, 3, CV_64F, iM);
    //Just to show you could have independently used F
            if( useUncalibrated == 2 )
                cvFindFundamentalMat( &_imagePoints1,
                &_imagePoints2, &_F);
            cvStereoRectifyUncalibrated( &_imagePoints1,
                &_imagePoints2, &_F,
                imageSize,
                &_H1, &_H2, 3);
            cvInvert(&_M1, &_iM);
            cvMatMul(&_H1, &_M1, &_R1);
            cvMatMul(&_iM, &_R1, &_R1);
            cvInvert(&_M2, &_iM);
            cvMatMul(&_H2, &_M2, &_R2);
            cvMatMul(&_iM, &_R2, &_R2);
    //Precompute map for cvRemap()
            cvInitUndistortRectifyMap(&_M1,&_D1,&_R1,&_M1,mx1,my1);

            cvInitUndistortRectifyMap(&_M2,&_D1,&_R2,&_M2,mx2,my2);
        }
        else
            assert(0);
        cvNamedWindow( "rectified", 1 );
// RECTIFY THE IMAGES AND FIND DISPARITY MAPS
        if( !isVerticalStereo )
            pair = cvCreateMat( imageSize.height, imageSize.width*2,
            CV_8UC3 );
        else
            pair = cvCreateMat( imageSize.height*2, imageSize.width,
            CV_8UC3 );
//Setup for finding stereo corrrespondences
        CvStereoBMState *BMState = cvCreateStereoBMState();
        assert(BMState != 0);
        BMState->preFilterSize=41;
        BMState->preFilterCap=31;
        BMState->SADWindowSize=41;
        BMState->minDisparity=-64;
        BMState->numberOfDisparities=128;
        BMState->textureThreshold=10;
        BMState->uniquenessRatio=15;
        for( i = 0; i < nframes; i++ )
        {
            IplImage *img1, *img2;
            
            if (calibrateOnlyLeft == true)
            {
                img1 = cvLoadImage(imageNames[0][i].c_str(),0);
                img2 = cvLoadImage(imageNames[0][i].c_str(),0);
            } else if (calibrateOnlyRight == true)
            {
                img1 = cvLoadImage(imageNames[1][i].c_str(),0);
                img2 = cvLoadImage(imageNames[1][i].c_str(),0);
            } else {
                img1 = cvLoadImage(imageNames[0][i].c_str(),0);
                img2 = cvLoadImage(imageNames[1][i].c_str(),0);
            }
            
            if( img1 && img2 )
            {
                CvMat part;
                cvRemap( img1, img1r, mx1, my1 );
                cvRemap( img2, img2r, mx2, my2 );
                if( !isVerticalStereo || useUncalibrated != 0 )
                {
              // When the stereo camera is oriented vertically,
              // useUncalibrated==0 does not transpose the
              // image, so the epipolar lines in the rectified
              // images are vertical. Stereo correspondence
              // function does not support such a case.
                    cvFindStereoCorrespondenceBM( img1r, img2r, disp,
                        BMState);
                    cvNormalize( disp, vdisp, 0, 256, CV_MINMAX );
                    cvNamedWindow( "disparity" );
                    cvShowImage( "disparity", vdisp );
                }
                if( !isVerticalStereo )
                {
                    cvGetCols( pair, &part, 0, imageSize.width );
                    cvCvtColor( img1r, &part, CV_GRAY2BGR );
                    cvGetCols( pair, &part, imageSize.width,
                        imageSize.width*2 );
                    cvCvtColor( img2r, &part, CV_GRAY2BGR );
                    for( j = 0; j < imageSize.height; j += 16 )
                        cvLine( pair, cvPoint(0,j),
                        cvPoint(imageSize.width*2,j),
                        CV_RGB(0,255,0));
                }
                else
                {
                    cvGetRows( pair, &part, 0, imageSize.height );
                    cvCvtColor( img1r, &part, CV_GRAY2BGR );
                    cvGetRows( pair, &part, imageSize.height,
                        imageSize.height*2 );
                    cvCvtColor( img2r, &part, CV_GRAY2BGR );
                    for( j = 0; j < imageSize.width; j += 16 )
                        cvLine( pair, cvPoint(j,0),
                        cvPoint(j,imageSize.height*2),
                        CV_RGB(0,255,0));
                }
                cvShowImage( "rectified", pair );
                if( cvWaitKey() == 27 )
                    break;
            }
            cvReleaseImage( &img1 );
            cvReleaseImage( &img2 );
        }
        cvReleaseStereoBMState(&BMState);
        cvReleaseMat( &mx1 );
        cvReleaseMat( &my1 );
        cvReleaseMat( &mx2 );
        cvReleaseMat( &my2 );
        cvReleaseMat( &img1r );
        cvReleaseMat( &img2r );
        cvReleaseMat( &disp );
    }
}
int main(int argc, char *argv[])
{
    int nx, ny;
    float squareSize;
    int fail = 0;
    //Check command line
    if (argc != 5 && argc != 6)
    {
        fprintf(stderr,"USAGE: %s imageList nx ny squareSize\n",argv[0]);
        fprintf(stderr,"\t imageList : Filename of the image list (string). Example : list.txt\n");
        fprintf(stderr,"\t nx : Number of horizontal squares (int > 0). Example : 9\n");
        fprintf(stderr,"\t ny : Number of vertical squares (int > 0). Example : 6\n");
        fprintf(stderr,"\t squareSize : Size of a square (float > 0). Example : 2.5\n");
        fprintf(stderr,"\t (optional) calibrate only a single camera, L for left, R for right. Example : L\n");
        return 1;
    }

		nx = atoi(argv[2]);
    ny = atoi(argv[3]);
    squareSize = (float)atof(argv[4]);

    // check for single-camera calibration
    if (argc == 6)
    {
        if (strcmp(argv[5], "L") == 0 || strcmp(argv[5], "l") == 0)
        {
            calibrateOnlyLeft = true;
        } else if (strcmp(argv[5], "R") == 0 || strcmp(argv[5], "r") == 0)
        {
            calibrateOnlyRight = true;
        } else {
            fprintf(stderr,"\nError: you specified a single camera calibration but it was not \"L\" or \"R\".\n");
            return 1;
        }
    }

    if (nx <= 0)
    {
        fail = 1;
        fprintf(stderr, "ERROR: nx value can not be <= 0\n");
    }
    if (ny <= 0)
    {
        fail = 1;
        fprintf(stderr, "ERROR: ny value can not be <= 0\n");
    }   
    if (squareSize <= 0.0)
    {
        fail = 1;
        fprintf(stderr, "ERROR: squareSize value can not be <= 0\n");
    }   

    if(fail != 0) return 1;

    StereoCalib(argv[1], nx, ny, 0, squareSize);
    return 0;
}
