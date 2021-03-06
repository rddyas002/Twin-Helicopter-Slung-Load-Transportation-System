% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 544.944810573879410 ; 544.835618115638000 ];

%-- Principal point:
cc = [ 324.928645069409410 ; 235.220014522915450 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.131113924307817 ; 0.193595699525474 ; -0.001982385268351 ; 0.001022677414090 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 0.769656979298378 ; 0.844900611371630 ];

%-- Principal point uncertainty:
cc_error = [ 1.255592210198739 ; 1.108505383577407 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.004844906664183 ; 0.016940218215671 ; 0.000527081642651 ; 0.000622966621578 ; 0.000000000000000 ];

%-- Image size:
nx = 640;
ny = 480;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 31;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -7.373321e-02 ; 2.969957e+00 ; 3.109403e-03 ];
Tc_1  = [ 4.712575e+01 ; -2.145476e+02 ; 8.652982e+02 ];
omc_error_1 = [ 1.201788e-03 ; 7.924473e-03 ; 9.579903e-03 ];
Tc_error_1  = [ 2.031820e+00 ; 1.763879e+00 ; 1.657345e+00 ];

%-- Image #2:
omc_2 = [ -1.062373e+00 ; 2.674911e+00 ; -1.557146e-01 ];
Tc_2  = [ 3.848549e+00 ; -4.832117e+01 ; 8.810563e+02 ];
omc_error_2 = [ 2.827001e-03 ; 8.049669e-03 ; 1.013710e-02 ];
Tc_error_2  = [ 2.041749e+00 ; 1.789168e+00 ; 1.660349e+00 ];

%-- Image #3:
omc_3 = [ -5.207309e-01 ; 2.827748e+00 ; 1.506719e-02 ];
Tc_3  = [ -4.988804e+01 ; -2.257793e+02 ; 9.465242e+02 ];
omc_error_3 = [ 2.737753e-03 ; 9.911140e-03 ; 1.156469e-02 ];
Tc_error_3  = [ 2.246587e+00 ; 1.945380e+00 ; 1.932796e+00 ];

%-- Image #4:
omc_4 = [ -2.755049e+00 ; 4.869672e-02 ; 7.048743e-01 ];
Tc_4  = [ 1.790730e+02 ; 1.493029e+02 ; 8.691344e+02 ];
omc_error_4 = [ 4.134540e-03 ; 1.315460e-03 ; 5.862336e-03 ];
Tc_error_4  = [ 2.073516e+00 ; 1.794500e+00 ; 1.360447e+00 ];

%-- Image #5:
omc_5 = [ -2.634481e+00 ; -7.754118e-02 ; 5.595383e-01 ];
Tc_5  = [ 1.060568e+02 ; 3.629184e+00 ; 8.909793e+02 ];
omc_error_5 = [ 4.022884e-03 ; 1.390699e-03 ; 5.780789e-03 ];
Tc_error_5  = [ 2.076527e+00 ; 1.826946e+00 ; 1.292649e+00 ];

%-- Image #6:
omc_6 = [ -2.939397e+00 ; -2.738304e-01 ; 2.654861e-01 ];
Tc_6  = [ -1.359239e+02 ; 2.285466e+02 ; 8.630557e+02 ];
omc_error_6 = [ 4.237137e-03 ; 1.197853e-03 ; 6.758662e-03 ];
Tc_error_6  = [ 2.044587e+00 ; 1.781524e+00 ; 1.512427e+00 ];

%-- Image #7:
omc_7 = [ -2.021957e+00 ; -2.227430e+00 ; 3.669591e-02 ];
Tc_7  = [ -3.178816e+02 ; -1.974038e+02 ; 7.515151e+02 ];
omc_error_7 = [ 3.302200e-03 ; 2.749286e-03 ; 6.068969e-03 ];
Tc_error_7  = [ 1.789232e+00 ; 1.607407e+00 ; 1.590212e+00 ];

%-- Image #8:
omc_8 = [ -2.208305e+00 ; -1.388995e+00 ; 4.080957e-01 ];
Tc_8  = [ -1.202203e+02 ; -4.402262e+01 ; 9.751029e+02 ];
omc_error_8 = [ 2.946173e-03 ; 2.197066e-03 ; 4.766136e-03 ];
Tc_error_8  = [ 2.252480e+00 ; 1.987694e+00 ; 1.607264e+00 ];

%-- Image #9:
omc_9 = [ -2.466795e+00 ; -1.180886e+00 ; 7.514043e-01 ];
Tc_9  = [ -4.533886e+01 ; 4.359276e+00 ; 9.889181e+02 ];
omc_error_9 = [ 3.432106e-03 ; 1.808496e-03 ; 5.161398e-03 ];
Tc_error_9  = [ 2.282432e+00 ; 2.008322e+00 ; 1.504875e+00 ];

%-- Image #10:
omc_10 = [ -1.768011e+00 ; 1.574266e+00 ; 7.955608e-02 ];
Tc_10  = [ -7.237793e+01 ; 1.422564e+02 ; 8.833037e+02 ];
omc_error_10 = [ 1.815180e-03 ; 2.621479e-03 ; 3.619585e-03 ];
Tc_error_10  = [ 2.070817e+00 ; 1.813543e+00 ; 1.332123e+00 ];

%-- Image #11:
omc_11 = [ -1.757094e+00 ; 1.657647e+00 ; 3.168564e-01 ];
Tc_11  = [ 1.362704e+02 ; 5.362614e+01 ; 8.948997e+02 ];
omc_error_11 = [ 2.013450e-03 ; 2.408828e-03 ; 3.719433e-03 ];
Tc_error_11  = [ 2.074208e+00 ; 1.836758e+00 ; 1.295633e+00 ];

%-- Image #12:
omc_12 = [ -2.246379e+00 ; 9.960188e-01 ; 6.607858e-01 ];
Tc_12  = [ 1.771244e+02 ; 6.615108e+01 ; 8.933205e+02 ];
omc_error_12 = [ 2.671940e-03 ; 1.824492e-03 ; 3.820316e-03 ];
Tc_error_12  = [ 2.067059e+00 ; 1.851578e+00 ; 1.319673e+00 ];

%-- Image #13:
omc_13 = [ -2.483942e+00 ; 9.583092e-02 ; 9.686882e-01 ];
Tc_13  = [ 1.400835e+02 ; 1.750662e+01 ; 8.868410e+02 ];
omc_error_13 = [ 3.073228e-03 ; 1.369305e-03 ; 3.903231e-03 ];
Tc_error_13  = [ 2.072109e+00 ; 1.826931e+00 ; 1.263155e+00 ];

%-- Image #14:
omc_14 = [ -2.670596e+00 ; -1.602014e-01 ; -1.032153e-01 ];
Tc_14  = [ -3.224882e+02 ; 4.160216e+01 ; 8.387685e+02 ];
omc_error_14 = [ 3.608318e-03 ; 1.305271e-03 ; 5.643149e-03 ];
Tc_error_14  = [ 1.974832e+00 ; 1.789851e+00 ; 1.571592e+00 ];

%-- Image #15:
omc_15 = [ -2.577506e+00 ; -3.205366e-02 ; 3.349811e-01 ];
Tc_15  = [ 2.558053e+01 ; -1.261425e+01 ; 9.293731e+02 ];
omc_error_15 = [ 3.789369e-03 ; 1.381531e-03 ; 5.374536e-03 ];
Tc_error_15  = [ 2.144053e+00 ; 1.902075e+00 ; 1.355627e+00 ];

%-- Image #16:
omc_16 = [ 2.928307e+00 ; -1.925018e-01 ; -5.332664e-02 ];
Tc_16  = [ -9.336742e+01 ; 2.047809e+02 ; 8.170519e+02 ];
omc_error_16 = [ 6.041133e-03 ; 1.244675e-03 ; 1.078820e-02 ];
Tc_error_16  = [ 1.917319e+00 ; 1.706221e+00 ; 1.954498e+00 ];

%-- Image #17:
omc_17 = [ 2.444770e+00 ; 1.274341e+00 ; -1.820278e-01 ];
Tc_17  = [ -3.694753e+02 ; -6.996293e+01 ; 7.427589e+02 ];
omc_error_17 = [ 2.527197e-03 ; 2.580974e-03 ; 4.648751e-03 ];
Tc_error_17  = [ 1.746488e+00 ; 1.613111e+00 ; 1.667856e+00 ];

%-- Image #18:
omc_18 = [ 3.006192e+00 ; -1.007949e-01 ; -8.106132e-01 ];
Tc_18  = [ 5.917070e+01 ; 8.779462e+01 ; 8.707244e+02 ];
omc_error_18 = [ 4.211001e-03 ; 1.247493e-03 ; 6.379209e-03 ];
Tc_error_18  = [ 2.017149e+00 ; 1.776282e+00 ; 1.512128e+00 ];

%-- Image #19:
omc_19 = [ -1.763471e+00 ; -1.880803e+00 ; -9.278683e-01 ];
Tc_19  = [ -1.062922e+02 ; -8.570433e+01 ; 2.694050e+02 ];
omc_error_19 = [ 1.293620e-03 ; 2.215586e-03 ; 2.997108e-03 ];
Tc_error_19  = [ 6.445234e-01 ; 5.726023e-01 ; 6.049689e-01 ];

%-- Image #20:
omc_20 = [ -2.364664e+00 ; -1.032786e+00 ; -6.628030e-02 ];
Tc_20  = [ -2.020083e+02 ; 3.792380e+01 ; 6.536252e+02 ];
omc_error_20 = [ 2.469170e-03 ; 1.628665e-03 ; 3.757439e-03 ];
Tc_error_20  = [ 1.518640e+00 ; 1.353491e+00 ; 1.096893e+00 ];

%-- Image #21:
omc_21 = [ -2.449948e+00 ; -1.768467e-02 ; 2.907350e-03 ];
Tc_21  = [ -1.066317e+02 ; 9.747463e+01 ; 6.460715e+02 ];
omc_error_21 = [ 2.382658e-03 ; 1.169701e-03 ; 3.418518e-03 ];
Tc_error_21  = [ 1.493321e+00 ; 1.324137e+00 ; 9.088260e-01 ];

%-- Image #22:
omc_22 = [ -2.489103e+00 ; 4.069685e-01 ; 4.485580e-01 ];
Tc_22  = [ -4.398300e+01 ; 1.563031e+02 ; 5.773025e+02 ];
omc_error_22 = [ 2.287967e-03 ; 1.312645e-03 ; 3.288305e-03 ];
Tc_error_22  = [ 1.350315e+00 ; 1.188307e+00 ; 7.958797e-01 ];

%-- Image #23:
omc_23 = [ 2.516857e+00 ; -1.136278e-02 ; -3.904008e-02 ];
Tc_23  = [ -9.706374e+01 ; 7.245195e+01 ; 2.834458e+02 ];
omc_error_23 = [ 2.106583e-03 ; 1.125852e-03 ; 3.141255e-03 ];
Tc_error_23  = [ 6.604826e-01 ; 5.980156e-01 ; 5.617013e-01 ];

%-- Image #24:
omc_24 = [ 1.897470e+00 ; 1.739840e+00 ; -6.130332e-01 ];
Tc_24  = [ -1.502691e+02 ; -3.979879e+01 ; 5.659222e+02 ];
omc_error_24 = [ 1.689871e-03 ; 2.115130e-03 ; 3.342497e-03 ];
Tc_error_24  = [ 1.308721e+00 ; 1.161721e+00 ; 8.794983e-01 ];

%-- Image #25:
omc_25 = [ 1.560877e+00 ; 1.425731e+00 ; -8.654028e-01 ];
Tc_25  = [ -1.609978e+02 ; -3.052007e+01 ; 4.750787e+02 ];
omc_error_25 = [ 1.630872e-03 ; 2.166878e-03 ; 2.505168e-03 ];
Tc_error_25  = [ 1.098785e+00 ; 9.832091e-01 ; 7.128670e-01 ];

%-- Image #26:
omc_26 = [ 2.120977e+00 ; 9.622762e-01 ; -6.014301e-01 ];
Tc_26  = [ -1.647818e+02 ; -3.753623e+01 ; 4.632370e+02 ];
omc_error_26 = [ 1.988209e-03 ; 1.862302e-03 ; 2.860508e-03 ];
Tc_error_26  = [ 1.077278e+00 ; 9.567550e-01 ; 8.115586e-01 ];

%-- Image #27:
omc_27 = [ 2.186495e+00 ; -2.115305e+00 ; -1.209912e-01 ];
Tc_27  = [ 7.769319e+01 ; 7.726143e+01 ; 4.670472e+02 ];
omc_error_27 = [ 2.302971e-03 ; 2.125977e-03 ; 4.779230e-03 ];
Tc_error_27  = [ 1.088597e+00 ; 9.553294e-01 ; 8.904942e-01 ];

%-- Image #28:
omc_28 = [ 8.141047e-02 ; 3.057719e+00 ; 2.201684e-01 ];
Tc_28  = [ 5.811217e+01 ; -1.314367e+02 ; 5.036813e+02 ];
omc_error_28 = [ 7.647242e-04 ; 3.856564e-03 ; 4.835932e-03 ];
Tc_error_28  = [ 1.175748e+00 ; 1.039196e+00 ; 9.699082e-01 ];

%-- Image #29:
omc_29 = [ 2.157795e+00 ; 2.155640e+00 ; 1.919288e-01 ];
Tc_29  = [ -1.353888e+02 ; -9.193592e+01 ; 4.684504e+02 ];
omc_error_29 = [ 2.461956e-03 ; 2.384346e-03 ; 5.050364e-03 ];
Tc_error_29  = [ 1.097294e+00 ; 9.738083e-01 ; 9.830038e-01 ];

%-- Image #30:
omc_30 = [ 2.845878e+00 ; 7.784162e-01 ; 2.955108e-02 ];
Tc_30  = [ -1.423653e+02 ; 4.428538e+01 ; 5.038303e+02 ];
omc_error_30 = [ 2.939865e-03 ; 1.128468e-03 ; 5.387354e-03 ];
Tc_error_30  = [ 1.175026e+00 ; 1.039513e+00 ; 1.031435e+00 ];

%-- Image #31:
omc_31 = [ 2.981050e+00 ; -1.171313e-02 ; 1.757151e-01 ];
Tc_31  = [ -1.575922e+02 ; 9.942342e+01 ; 4.914823e+02 ];
omc_error_31 = [ 3.047183e-03 ; 8.077266e-04 ; 5.428436e-03 ];
Tc_error_31  = [ 1.147667e+00 ; 1.034441e+00 ; 1.056849e+00 ];

