How to use this code:

<1> make sure you have opencv library installed. If you have it, please go to step <2>; if not, follow the tips below:
   - sudo apt-get install libopencv-dev
   - All the header files will be installed in /usr/include/opencv by default
   - All the libraries will be installed in /usr/lib by default
   - go to step <3>
<2> if you have opencv already installed from source code, please modify the Makefile as:
   - INCS= your correct opencv include path, e.g. /usr/local/include/opencv (which is the default path of opencv library)
   - LIBS= your correct opencv lib path, e.g. /usr/local/lib
<3> make clean;
<4> make;
<5> copy the generated FindCorners.exe into the "autoCornerFinder" folder in your toolbox path, replacing the existing one.
<6> test it by the "Extract grid corners" button in the toolbox, after "Load images". 

NOTE THAT THE CODE WORKS ONLY WITH JPEG IMAGES.
Note, if you want to display all the processing steps of the corner extraction, just change the flag VIS in cvcalibinit3.cpp to 1.
Please observe that the exetutable is called withing the "click_ima_calib_rufli.m". in the toolbox path.

If you use this code, please cite the following articles:

1. Scaramuzza, D., Martinelli, A. and Siegwart, R. (2006), A Toolbox for Easily Calibrating Omnidirectional Cameras, Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems  (IROS 2006), Beijing, China, October 2006.

2. Scaramuzza, D., Martinelli, A. and Siegwart, R., (2006). "A Flexible Technique for Accurate Omnidirectional Camera Calibration and Structure from Motion", Proceedings of IEEE International Conference of Vision Systems  (ICVS'06), New York, January 5-7, 2006.

3. Rufli, M., Scaramuzza, D., and Siegwart, R. (2008), Automatic Detection of Checkerboards on Blurred and Distorted Images, Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2008), Nice, France, September 2008.


