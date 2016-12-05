classdef C_calib_data < handle
    %CALIB_DATA Stores image and calibration data used by the calibration toolbox
    
    properties
        
        % Image data
        
        calib_name          % Base image name
        format_image        % Image format
        L                   % Image file names
        
        map                 % Colormap for displaying images

        I                   % Calibration images
        n_ima               % Number of images read
        active_images       % Vector indicating images used
        ind_active          % Indices of images used
        ind_read            % Indices of images read
        ima_proc            % Images being processed
        
        % Calibration model
        
        ocam_model =...
            struct( 'ss',[],... % Coefficients of polynomial
                    'xc',[],... % x-coordinate of image center
                    'yc',[],... % y-coordinate of image center
                    'c',[],'d',[],'e',[],... % Affine transformation parameters
                    'width',[],...  % Image width
                    'height',[])    % Image height

        RRfin               % Extrinsic parameters of checkerboards
      
        taylor_order        % Order of the polynomial
        taylor_order_default
        
        dX                  % Width of a square on checkerboard (mm)
        dY                  % Height of a square on checkerboard (mm)
        n_sq_x              % Number of squares in x-direction
        n_sq_y              % Number of squares in y-direction
        Xt                  % Checkerboard corner coordinates (mm)
        Yt
        Xp_abs              % Checkerboard corner coordinates (px)
        Yp_abs
        
        wintx               % Size of corner search window for assisted
        winty               % manual corner selection
        
        % Flags
        no_image_file       % Indicates missing image files
        calibrated          % Indicates calibrated ocam_model
        
        
    end
    
    methods
        
       
    end
    
end

