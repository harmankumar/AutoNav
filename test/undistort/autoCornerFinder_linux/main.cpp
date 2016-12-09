/************************************************************************************\
    This is improved variant of chessboard corner detection algorithm that
    uses a graph of connected quads. It is based on the code contributed
    by Vladimir Vezhnevets and Philip Gruebele.
    Here is the copyright notice from the original Vladimir's code:
    ===============================================================

    The algorithms developed and implemented by Vezhnevets Vldimir
    aka Dead Moroz (vvp@graphics.cs.msu.ru)
    See http://graphics.cs.msu.su/en/research/calibration/opencv.html
    for detailed information.

    Reliability additions and modifications made by Philip Gruebele.
    <a href="mailto:pgruebele@cox.net">pgruebele@cox.net</a>

	His code was adapted for use with low resolution and omnidirectional cameras
	by Martin Rufli during his Master Thesis under supervision of Davide Scaramuzza, at the ETH Zurich. Further enhancements include:
		- Increased chance of correct corner matching.
		- Corner matching over all dilation runs.
		
If you use this code, please cite the following articles:

1. Scaramuzza, D., Martinelli, A. and Siegwart, R. (2006), A Toolbox for Easily Calibrating Omnidirectional Cameras, Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems  (IROS 2006), Beijing, China, October 2006.
2. Scaramuzza, D., Martinelli, A. and Siegwart, R., (2006). "A Flexible Technique for Accurate Omnidirectional Camera Calibration and Structure from Motion", Proceedings of IEEE International Conference of Vision Systems  (ICVS'06), New York, January 5-7, 2006.
3. Rufli, M., Scaramuzza, D., and Siegwart, R. (2008), Automatic Detection of Checkerboards on Blurred and Distorted Images, Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2008), Nice, France, September 2008.

\************************************************************************************/


// Includes
#include <cstdlib>
#include <iostream>

#include <opencv.hpp>

#include <stdio.h>
#include <string.h>
#include <time.h>

#include <fstream>
using namespace std;
using std::ifstream;

#include "cvcalibinit3.h"
/*
static int cvFindChessboardCorners3( const void* arr, CvSize pattern_size,
                             CvPoint2D32f* out_corners, int* out_corner_count,
                             int min_number_of_corners );
*/	

//===========================================================================
// MAIN LOOP 
//===========================================================================
int main( int argc, char** argv )
{
	// CHOOSE METHOD (0= Vladimir Vezhnevets, 1= Martin Rufli)
	int method = 1;


	// Initializations
	CvSize board_size				= {7,6};
	const char* input_filename		= 0;
	CvCapture* capture				= 0;
	FILE* f							= 0;
	char imagename[1024];
	CvMemStorage* storage;
	CvSeq* image_points_seq			= 0;
	int elem_size;
	CvPoint2D32f* image_points_buf	= 0;
	CvSize img_size					= {0,0};
	int found						= -2;
	int min_number_of_corners		= 42;
	input_filename					= "pictures.txt";
	//input_filename					= "myVideo2.avi";

	// Create error message file
	ofstream error("outputImages/error.txt");


	// Read the "argv" function input arguments
	for(int i = 1; i < argc; i++ )
	{
		const char* s = argv[i];
		if( strcmp( s, "-w" ) == 0 )
		{
			if( sscanf( argv[++i], "%u", &board_size.width ) != 1 || board_size.width <= 0 )
			{
				error << "Invalid board width" << endl;
				error.close();
				return -1;
			}
		}
		else if( strcmp( s, "-h" ) == 0 )
		{
			if( sscanf( argv[++i], "%u", &board_size.height ) != 1 || board_size.height <= 0 )
			{
				error << "Invalid board height" << endl;
				error.close();
				return -1;
			}
		}
		else if( strcmp( s, "-m" ) == 0 )
		{
			if( sscanf( argv[++i], "%u", &min_number_of_corners ) != 1 )
			{
				error << "Invalid minimal number of corners" << endl;
				error.close();
				return -1;
			}
		}
		else if( s[0] != '-' )
			input_filename = s;
		else
		{
				error << "Unknown option" << endl;
				error.close();
				return -1;
		}
	}


	// Close error message file
	error.close();


	// Figure out what kind of image input needs to be prepared
	if( input_filename )
	{
		// Try to open a video sequence
		//capture = cvCreateFileCapture( input_filename ); //OBRAND commented out
		if( !capture )
		{
			// Try to open an input image
			f = fopen( input_filename, "rt" );
			if( !f )
				return fprintf( stderr, "The input file could not be opened\n" ), -1;
		}
	}
	else
		// Open a live video stream
		capture = cvCreateCameraCapture(0);


	// Nothing could be opened -> error
	if( !capture && !f )
		return fprintf( stderr, "Could not initialize video capture\n" ), -2;

	
	// Allocate memory
	elem_size = board_size.width*board_size.height*sizeof(image_points_buf[0]);
	storage = cvCreateMemStorage( MAX( elem_size*4, 1 << 16 ));
	image_points_buf = (CvPoint2D32f*)cvAlloc( elem_size );
	image_points_seq = cvCreateSeq( 0, sizeof(CvSeq), elem_size, storage );


	// For loop which goes through all images specified above
	for(int j = 1;; j++)
	{
		// Initializations
		IplImage *view = 0, *view_gray = 0;
		int count = 0, blink = 0;
		CvSize text_size = {0,0};
		int base_line = 0;
		// Load the correct image...
		if( f && fgets( imagename, sizeof(imagename)-2, f ))
		{

			int l = (int) strlen(imagename);
			if( l > 0 && imagename[l-1] == '\n' )
				imagename[--l] = '\0';
			if( l > 0 )
			{
				if( imagename[0] == '#' )
					continue;
				// Load as BGR 3 channel image
				view = cvLoadImage( imagename, 1 );
				// Currently the following file formats are supported: 
				// Windows bitmaps				BMP, DIB
				// JPEG files					JPEG, JPG, JPE
				// Portable Network Graphics	PNG
				// Portable image format		PBM, PGM, PPM
				// Sun rasters					SR, RAS
				// TIFF files					TIFF, TIF
				// NOTABLY: GIF IS NOT SUPPORTED!
			}
		}


		// ...Or capture the correct frame from the video
		else if( capture )
		{
			IplImage* view0 = cvQueryFrame( capture );
			if( view0 )
			{
				view = cvCreateImage( cvGetSize(view0), IPL_DEPTH_8U, view0->nChannels );
				if( view0->origin == IPL_ORIGIN_BL )
					cvFlip( view0, view, 0 );
				else
					cvCopy( view0, view );
			}
		}

		
		// If no more images are to be processed -> break
		if( !view)
		{
			break;
		}

		// If esc key was pressed -> break
		int key = cvWaitKey(10);
		if( key == 27)
		{
			break;
		}

		img_size = cvGetSize(view);
		

		// Perform the corner finding algorithm
		// 0 = old method, 1 = New method by Martin Rufli
		if (method == 0)
		{
			//found = cvFindChessboardCorners1( view, board_size,
			//		image_points_buf, &count, CV_CALIB_CB_ADAPTIVE_THRESH );
		}
		else
		{
			found = cvFindChessboardCorners3( view, board_size,
					image_points_buf, &count, min_number_of_corners );
		}

		if( !view )
			break;
		cvReleaseImage( &view );
	}

	if( capture )
		cvReleaseCapture( &capture );
	
	return found;
}
