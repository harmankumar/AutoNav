/*------------------------------------------------------------------------------
   Example code that shows the use of the 'cam2world" and 'world2cam" functions
   Shows also how to undistort images into perspective or panoramic images
   
   NOTE, IF YOU WANT TO SPEED UP THE REMAP FUNCTION I STRONGLY RECOMMEND TO INSTALL
   INTELL IPP LIBRARIES ( http://software.intel.com/en-us/intel-ipp/ )
   YOU JUST NEED TO INSTALL IT AND INCLUDE ipp.h IN YOUR PROGRAM
   
   Copyright (C) 2009 DAVIDE SCARAMUZZA, ETH Zurich  
   Author: Davide Scaramuzza - email: davide.scaramuzza@ieee.org
------------------------------------------------------------------------------*/

#include "ocam_functions.h"

int main(int argc, char *argv[])
{   
  /* --------------------------------------------------------------------*/
  /* Read the parameters of the omnidirectional camera from the TXT file */
  /* --------------------------------------------------------------------*/
  struct ocam_model o, o_cata; // our ocam_models for the fisheye and catadioptric cameras
  get_ocam_model(&o, "./calib_results_fisheye.txt");
  get_ocam_model(&o_cata, "./calib_results_catadioptric.txt");    
  /* --------------------------------------------------------------------*/    
  /* Print ocam_model parameters                                         */
  /* --------------------------------------------------------------------*/  
  int i;
  printf("pol =\n");    for (i=0; i<o.length_pol; i++){    printf("\t%e\n",o.pol[i]); };    printf("\n");
  printf("invpol =\n"); for (i=0; i<o.length_invpol; i++){ printf("\t%e\n",o.invpol[i]); }; printf("\n");  
  printf("\nxc = %f\nyc = %f\n\nwidth = %d\nheight = %d\n",o.xc,o.yc,o.width,o.height);

  /* --------------------------------------------------------------------*/
  /* WORLD2CAM projects 3D point into the image                          */
  /* NOTE!!! The coordinates are expressed according the C convention,   */
  /* that is, from the origin (0,0) instead than from 1 (MATLAB).        */
  /* --------------------------------------------------------------------*/
  double point3D[3] = { 100 , 200 , -300 };       // a sample 3D point
  double point2D[2];                              // the image point in pixel coordinates  
  world2cam(point2D, point3D, &o); // The behaviour of this function is the same as in MATLAB
  
  /* --------------------------------------------------------------------*/  
  /* Display re-projected coordinates                                    */
  /* --------------------------------------------------------------------*/  
  printf("\nworld2cam: pixel coordinates reprojected onto the image\n");  
  printf("m_row= %2.4f, m_col=%2.4f\n", point2D[0], point2D[1]);

  /* --------------------------------------------------------------------*/
  /* CAM2WORLD back-projects pixel points on to the unit sphere          */
  /* The behaviour of this function is the same as in MATLAB             */
  /* --------------------------------------------------------------------*/

  cam2world(point3D, point2D, &o); 

  /* --------------------------------------------------------------------*/  
  /* Display back-projected normalized coordinates (on the unit sphere)  */
  /* --------------------------------------------------------------------*/  
  printf("\ncam2world: coordinates back-projected onto the unit sphere (x^2+y^2+z^2=1)\n");
  printf("x= %2.4f, y=%2.4f, z=%2.4f\n", point3D[0], point3D[1], point3D[2]);
  
  /* --------------------------------------------------------------------*/  
  /* Allocate space for the unistorted images                            */
  /* --------------------------------------------------------------------*/  
  IplImage *src1         = cvLoadImage("./test_fisheye.jpg");      // source image 1
  IplImage *src2         = cvLoadImage("./test_catadioptric.jpg");      // source image 2  
  IplImage *dst_persp   = cvCreateImage( cvGetSize(src1), 8, 3 );   // undistorted perspective and panoramic image
  CvSize size_pan_image = cvSize(1200,400);        // size of the undistorted panoramic image
  IplImage *dst_pan     = cvCreateImage( size_pan_image, 8, 3 );    // undistorted panoramic image

  CvMat* mapx_persp = cvCreateMat(src1->height, src1->width, CV_32FC1);
  CvMat* mapy_persp = cvCreateMat(src1->height, src1->width, CV_32FC1);
  CvMat* mapx_pan   = cvCreateMat(dst_pan->height, dst_pan->width, CV_32FC1);
  CvMat* mapy_pan   = cvCreateMat(dst_pan->height, dst_pan->width, CV_32FC1);
  
  /* --------------------------------------------------------------------  */  
  /* Create Look-Up-Table for perspective undistortion                     */
  /* SF is kind of distance from the undistorted image to the camera       */
  /* (it is not meters, it is justa zoom fator)                            */
  /* Try to change SF to see how it affects the result                     */
  /* The undistortion is done on a  plane perpendicular to the camera axis */
  /* --------------------------------------------------------------------  */
  float sf = 4;
  create_perspecive_undistortion_LUT( mapx_persp, mapy_persp, &o, sf );

  /* --------------------------------------------------------------------  */  
  /* Create Look-Up-Table for panoramic undistortion                       */
  /* The undistortoin is just a simple cartesia-to-polar transformation    */
  /* Note, only the knowledge of image center (xc,yc) is used to undisort the image      */
  /* xc, yc are the row and column coordinates of the image center         */
  /* Note, if you would like to flip the image, just inverte the sign of theta in this function */
  /* --------------------------------------------------------------------  */  
  float Rmax = 470;  // the maximum radius of the region you would like to undistort into a panorama
  float Rmin = 20;   // the minimum radius of the region you would like to undistort into a panorama  
  create_panoramic_undistortion_LUT ( mapx_pan, mapy_pan, Rmin, Rmax, o_cata.xc, o_cata.yc);  

  /* --------------------------------------------------------------------*/  
  /* Undistort using specified interpolation method                      */
  /* Other possible values are (see OpenCV doc):                         */
  /* CV_INTER_NN - nearest-neighbor interpolation,                       */
  /* CV_INTER_LINEAR - bilinear interpolation (used by default)          */
  /* CV_INTER_AREA - resampling using pixel area relation. It is the preferred method for image decimation that gives moire-free results. In case of zooming it is similar to CV_INTER_NN method. */
  /* CV_INTER_CUBIC - bicubic interpolation.                             */
  /* --------------------------------------------------------------------*/
  cvRemap( src1, dst_persp, mapx_persp, mapy_persp, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, cvScalarAll(0) );
  cvRemap( src2, dst_pan  , mapx_pan, mapy_pan, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, cvScalarAll(0) );  
  
  /* --------------------------------------------------------------------*/
  /* Display image                                                       */
  /* --------------------------------------------------------------------*/  
  cvNamedWindow( "Original fisheye camera image", 1 );
  cvShowImage( "Original fisheye camera image", src1 );

  cvNamedWindow( "Undistorted Perspective Image", 1 );
  cvShowImage( "Undistorted Perspective Image", dst_persp );
  
  cvNamedWindow( "Original Catadioptric camera image", 1 );
  cvShowImage( "Original Catadioptric camera image", src2 );

  cvNamedWindow( "Undistorted Panoramic Image", 1 );
  cvShowImage( "Undistorted Panoramic Image", dst_pan );

  /* --------------------------------------------------------------------*/    
  /* Save image                                                          */
  /* --------------------------------------------------------------------*/  
  cvSaveImage("undistorted_perspective.jpg",dst_persp);
  printf("\nImage %s saved\n","undistorted_perspective.jpg");

  cvSaveImage("undistorted_panoramic.jpg",dst_pan);
  printf("\nImage %s saved\n","undistorted_panoramic.jpg");

  /* --------------------------------------------------------------------*/    
  /* Wait until key presses                                              */
  /* --------------------------------------------------------------------*/  
  cvWaitKey();

  /* --------------------------------------------------------------------*/    
  /* Free memory                                                         */
  /* --------------------------------------------------------------------*/  
  cvReleaseImage(&src1);
  cvReleaseImage(&src2);  
  cvReleaseImage(&dst_persp);
  cvReleaseImage(&dst_pan);  
  cvReleaseMat(&mapx_persp);
  cvReleaseMat(&mapy_persp);  
  cvReleaseMat(&mapx_pan);
  cvReleaseMat(&mapy_pan);    

  return 0;
}
