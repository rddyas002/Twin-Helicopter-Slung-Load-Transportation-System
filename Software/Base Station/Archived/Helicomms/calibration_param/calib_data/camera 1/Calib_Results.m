% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 540.448746060631490 ; 540.183159661151190 ];

%-- Principal point:
cc = [ 292.023060594315670 ; 256.458011762466010 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.119217611558708 ; 0.140890850038588 ; -0.000334557682968 ; -0.000821103146540 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 1.089594274300245 ; 1.096009223147002 ];

%-- Principal point uncertainty:
cc_error = [ 0.970328120416606 ; 1.001137300530257 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.004028354169647 ; 0.014433834564859 ; 0.000445375317349 ; 0.000430653904654 ; 0.000000000000000 ];

%-- Image size:
nx = 640;
ny = 480;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 35;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 2.944843e+00 ; 9.485941e-01 ; -3.411475e-01 ];
Tc_1  = [ -2.042915e+02 ; 3.732823e+01 ; 6.020574e+02 ];
omc_error_1 = [ 2.492910e-03 ; 1.419699e-03 ; 4.473018e-03 ];
Tc_error_1  = [ 1.080148e+00 ; 1.131206e+00 ; 1.322813e+00 ];

%-- Image #2:
omc_2 = [ -3.063333e+00 ; -2.373956e-01 ; 3.311597e-01 ];
Tc_2  = [ -1.004175e+02 ; 1.043867e+02 ; 5.127691e+02 ];
omc_error_2 = [ 2.465580e-03 ; 5.721894e-04 ; 4.236040e-03 ];
Tc_error_2  = [ 9.178191e-01 ; 9.479717e-01 ; 1.120738e+00 ];

%-- Image #3:
omc_3 = [ 2.242775e+00 ; -2.143138e+00 ; -5.984795e-02 ];
Tc_3  = [ 1.531844e+02 ; 9.606601e+01 ; 4.294792e+02 ];
omc_error_3 = [ 1.765841e-03 ; 1.777498e-03 ; 3.882401e-03 ];
Tc_error_3  = [ 7.855167e-01 ; 8.171414e-01 ; 9.855965e-01 ];

%-- Image #4:
omc_4 = [ 2.444485e+00 ; -1.330818e+00 ; -2.210671e-01 ];
Tc_4  = [ 9.507149e+01 ; 9.705164e+01 ; 4.363367e+02 ];
omc_error_4 = [ 1.902248e-03 ; 1.375591e-03 ; 3.263225e-03 ];
Tc_error_4  = [ 7.985268e-01 ; 8.343853e-01 ; 1.041647e+00 ];

%-- Image #5:
omc_5 = [ 2.802502e+00 ; -2.373014e-01 ; 3.775953e-01 ];
Tc_5  = [ -1.015518e+02 ; 1.250066e+02 ; 5.925601e+02 ];
omc_error_5 = [ 2.741014e-03 ; 9.350028e-04 ; 4.158703e-03 ];
Tc_error_5  = [ 1.078761e+00 ; 1.128843e+00 ; 1.508758e+00 ];

%-- Image #6:
omc_6 = [ 2.630032e+00 ; 4.743499e-01 ; -7.305989e-01 ];
Tc_6  = [ -8.326970e+01 ; 3.776459e+01 ; 5.413108e+02 ];
omc_error_6 = [ 2.070874e-03 ; 1.157666e-03 ; 2.789753e-03 ];
Tc_error_6  = [ 9.696581e-01 ; 9.968931e-01 ; 1.127582e+00 ];

%-- Image #7:
omc_7 = [ -6.709212e-01 ; -2.932609e+00 ; 4.833515e-01 ];
Tc_7  = [ 2.342157e+02 ; -4.145276e+01 ; 8.596554e+02 ];
omc_error_7 = [ 1.610383e-03 ; 5.938569e-03 ; 8.014577e-03 ];
Tc_error_7  = [ 1.566167e+00 ; 1.631167e+00 ; 1.799411e+00 ];

%-- Image #8:
omc_8 = [ 2.729165e-01 ; 3.059159e+00 ; -5.709733e-01 ];
Tc_8  = [ 1.021600e+02 ; -5.331685e+01 ; 9.470593e+02 ];
omc_error_8 = [ 1.584772e-03 ; 5.037200e-03 ; 6.864716e-03 ];
Tc_error_8  = [ 1.695716e+00 ; 1.765986e+00 ; 1.842624e+00 ];

%-- Image #9:
omc_9 = [ -1.740814e+00 ; -2.487700e+00 ; 4.503712e-01 ];
Tc_9  = [ -2.497219e+02 ; -1.230021e+02 ; 9.671910e+02 ];
omc_error_9 = [ 2.392147e-03 ; 2.977659e-03 ; 5.394576e-03 ];
Tc_error_9  = [ 1.760473e+00 ; 1.827755e+00 ; 2.062661e+00 ];

%-- Image #10:
omc_10 = [ -2.488566e+00 ; -1.213476e+00 ; 5.920165e-02 ];
Tc_10  = [ -3.520478e+02 ; -7.683996e+01 ; 9.149321e+02 ];
omc_error_10 = [ 3.218719e-03 ; 1.434900e-03 ; 4.858132e-03 ];
Tc_error_10  = [ 1.675424e+00 ; 1.757602e+00 ; 2.028314e+00 ];

%-- Image #11:
omc_11 = [ -2.528422e+00 ; 6.678894e-01 ; 2.627704e-01 ];
Tc_11  = [ -8.710381e+00 ; 9.211698e+01 ; 9.030574e+02 ];
omc_error_11 = [ 2.882908e-03 ; 1.419583e-03 ; 4.105459e-03 ];
Tc_error_11  = [ 1.617635e+00 ; 1.686654e+00 ; 1.723317e+00 ];

%-- Image #12:
omc_12 = [ -2.260288e+00 ; 1.344033e+00 ; 3.986831e-01 ];
Tc_12  = [ 7.702303e+01 ; 6.795089e+01 ; 8.032371e+02 ];
omc_error_12 = [ 2.363343e-03 ; 1.857903e-03 ; 3.756797e-03 ];
Tc_error_12  = [ 1.434125e+00 ; 1.500085e+00 ; 1.485183e+00 ];

%-- Image #13:
omc_13 = [ -2.840764e+00 ; 5.739435e-02 ; 7.765045e-01 ];
Tc_13  = [ 3.276683e-01 ; 1.045111e+02 ; 7.855045e+02 ];
omc_error_13 = [ 2.890700e-03 ; 1.017696e-03 ; 4.034377e-03 ];
Tc_error_13  = [ 1.415436e+00 ; 1.452721e+00 ; 1.532061e+00 ];

%-- Image #14:
omc_14 = [ 2.857276e+00 ; -6.982783e-01 ; -7.842366e-01 ];
Tc_14  = [ -1.484855e+00 ; 1.950828e+02 ; 8.154395e+02 ];
omc_error_14 = [ 3.032266e-03 ; 9.970308e-04 ; 4.335627e-03 ];
Tc_error_14  = [ 1.481676e+00 ; 1.512605e+00 ; 1.839980e+00 ];

%-- Image #15:
omc_15 = [ -3.070905e+00 ; -3.878646e-01 ; 2.473951e-01 ];
Tc_15  = [ -1.461363e+02 ; 8.655715e+01 ; 9.295874e+02 ];
omc_error_15 = [ 5.530023e-03 ; 1.061227e-03 ; 9.706821e-03 ];
Tc_error_15  = [ 1.669718e+00 ; 1.728034e+00 ; 2.127312e+00 ];

%-- Image #16:
omc_16 = [ -2.627220e+00 ; -9.988533e-01 ; -1.935554e-01 ];
Tc_16  = [ -3.741018e+02 ; 4.609072e+01 ; 9.649192e+02 ];
omc_error_16 = [ 3.787550e-03 ; 1.528746e-03 ; 5.666775e-03 ];
Tc_error_16  = [ 1.764594e+00 ; 1.858693e+00 ; 2.233544e+00 ];

%-- Image #17:
omc_17 = [ -2.610060e+00 ; 7.520422e-01 ; 1.778278e-01 ];
Tc_17  = [ 2.816612e+02 ; 4.498102e+01 ; 9.876178e+02 ];
omc_error_17 = [ 3.661028e-03 ; 1.164651e-03 ; 5.261663e-03 ];
Tc_error_17  = [ 1.822298e+00 ; 1.889207e+00 ; 2.030095e+00 ];

%-- Image #18:
omc_18 = [ -2.589020e+00 ; -3.979721e-02 ; 2.079622e-01 ];
Tc_18  = [ 1.399501e+02 ; -2.381585e+01 ; 1.000721e+03 ];
omc_error_18 = [ 3.666392e-03 ; 1.408586e-03 ; 5.272087e-03 ];
Tc_error_18  = [ 1.822450e+00 ; 1.873655e+00 ; 1.962616e+00 ];

%-- Image #19:
omc_19 = [ -2.577048e+00 ; -1.151786e-03 ; 1.649451e-01 ];
Tc_19  = [ -7.703269e+01 ; -2.581774e+01 ; 9.460659e+02 ];
omc_error_19 = [ 3.377795e-03 ; 1.176646e-03 ; 4.780067e-03 ];
Tc_error_19  = [ 1.695375e+00 ; 1.765818e+00 ; 1.817778e+00 ];

%-- Image #20:
omc_20 = [ -2.081076e+00 ; 1.857578e+00 ; 2.174487e-01 ];
Tc_20  = [ 1.261638e+02 ; -1.429406e+01 ; 8.345899e+02 ];
omc_error_20 = [ 3.301288e-03 ; 3.022259e-03 ; 6.117496e-03 ];
Tc_error_20  = [ 1.489844e+00 ; 1.558956e+00 ; 1.636644e+00 ];

%-- Image #21:
omc_21 = [ 1.283585e+00 ; 2.735334e+00 ; -4.732620e-02 ];
Tc_21  = [ -7.225502e+01 ; -2.463018e+02 ; 7.297505e+02 ];
omc_error_21 = [ 1.716161e-03 ; 4.161587e-03 ; 6.159859e-03 ];
Tc_error_21  = [ 1.333870e+00 ; 1.358248e+00 ; 1.615931e+00 ];

%-- Image #22:
omc_22 = [ -1.674631e+00 ; -2.635681e+00 ; 2.586626e-01 ];
Tc_22  = [ -1.683165e+01 ; -1.541933e+02 ; 7.889151e+02 ];
omc_error_22 = [ 2.785382e-03 ; 4.453369e-03 ; 7.792571e-03 ];
Tc_error_22  = [ 1.424051e+00 ; 1.457396e+00 ; 1.725997e+00 ];

%-- Image #23:
omc_23 = [ 2.373338e+00 ; 2.005819e+00 ; -2.803705e-01 ];
Tc_23  = [ -1.453150e+02 ; -5.581153e+01 ; 7.695144e+02 ];
omc_error_23 = [ 3.723573e-03 ; 3.143016e-03 ; 7.453992e-03 ];
Tc_error_23  = [ 1.379850e+00 ; 1.428905e+00 ; 1.686610e+00 ];

%-- Image #24:
omc_24 = [ -2.765931e+00 ; -1.433588e+00 ; 1.599341e-01 ];
Tc_24  = [ -2.014122e+02 ; -2.473042e+01 ; 7.393778e+02 ];
omc_error_24 = [ 3.974042e-03 ; 1.875710e-03 ; 7.606145e-03 ];
Tc_error_24  = [ 1.327410e+00 ; 1.383236e+00 ; 1.666956e+00 ];

%-- Image #25:
omc_25 = [ 3.006149e+00 ; 8.030107e-01 ; -1.238272e-01 ];
Tc_25  = [ -2.119917e+02 ; 8.417735e+01 ; 6.931513e+02 ];
omc_error_25 = [ 3.771274e-03 ; 1.595028e-03 ; 7.193904e-03 ];
Tc_error_25  = [ 1.249436e+00 ; 1.304623e+00 ; 1.599536e+00 ];

%-- Image #26:
omc_26 = [ 3.050286e+00 ; 6.923433e-01 ; -1.595569e-01 ];
Tc_26  = [ -1.770535e+02 ; 6.968908e+01 ; 5.964110e+02 ];
omc_error_26 = [ 3.047396e-03 ; 1.170191e-03 ; 5.744611e-03 ];
Tc_error_26  = [ 1.072009e+00 ; 1.116977e+00 ; 1.359282e+00 ];

%-- Image #27:
omc_27 = [ -3.121935e+00 ; 3.329047e-02 ; 1.025860e-01 ];
Tc_27  = [ -1.719975e+02 ; 1.215622e+02 ; 5.375279e+02 ];
omc_error_27 = [ 2.690234e-03 ; 7.615045e-04 ; 4.856944e-03 ];
Tc_error_27  = [ 9.694740e-01 ; 1.014788e+00 ; 1.237507e+00 ];

%-- Image #28:
omc_28 = [ 2.769666e+00 ; 6.530017e-01 ; 5.698694e-01 ];
Tc_28  = [ -4.651440e+01 ; 5.758317e+01 ; 4.671400e+02 ];
omc_error_28 = [ 2.293648e-03 ; 8.108285e-04 ; 3.191256e-03 ];
Tc_error_28  = [ 8.545715e-01 ; 8.677795e-01 ; 1.170953e+00 ];

%-- Image #29:
omc_29 = [ -2.488574e+00 ; 5.538334e-01 ; -3.899251e-01 ];
Tc_29  = [ 3.244999e+01 ; 1.705092e+02 ; 5.332295e+02 ];
omc_error_29 = [ 2.039408e-03 ; 9.119358e-04 ; 2.681866e-03 ];
Tc_error_29  = [ 9.644474e-01 ; 9.945586e-01 ; 1.026499e+00 ];

%-- Image #30:
omc_30 = [ -2.720101e+00 ; -9.877422e-01 ; 7.637893e-01 ];
Tc_30  = [ -1.524583e+02 ; 3.082701e+01 ; 5.620526e+02 ];
omc_error_30 = [ 2.219662e-03 ; 6.956603e-04 ; 3.113744e-03 ];
Tc_error_30  = [ 1.010433e+00 ; 1.040665e+00 ; 1.072618e+00 ];

%-- Image #31:
omc_31 = [ -2.234182e+00 ; 1.850171e+00 ; 1.020369e+00 ];
Tc_31  = [ 1.245115e+02 ; 8.068947e+01 ; 4.419511e+02 ];
omc_error_31 = [ 1.418883e-03 ; 1.847022e-03 ; 3.024053e-03 ];
Tc_error_31  = [ 7.971472e-01 ; 8.298054e-01 ; 8.986387e-01 ];

%-- Image #32:
omc_32 = [ -2.151053e+00 ; 2.259948e+00 ; 1.537234e-01 ];
Tc_32  = [ 1.985721e+02 ; 6.077548e+01 ; 3.986312e+02 ];
omc_error_32 = [ 1.695704e-03 ; 1.535226e-03 ; 3.675747e-03 ];
Tc_error_32  = [ 7.273114e-01 ; 7.686449e-01 ; 9.215448e-01 ];

%-- Image #33:
omc_33 = [ -6.945757e-03 ; 3.054098e+00 ; -9.777701e-02 ];
Tc_33  = [ 5.548284e+01 ; -1.661461e+02 ; 4.292611e+02 ];
omc_error_33 = [ 4.956008e-04 ; 2.421708e-03 ; 3.224656e-03 ];
Tc_error_33  = [ 7.833472e-01 ; 7.932735e-01 ; 9.262624e-01 ];

%-- Image #34:
omc_34 = [ 2.125915e+00 ; 2.068588e+00 ; 3.006581e-02 ];
Tc_34  = [ -8.963612e+01 ; -1.232536e+02 ; 4.044048e+02 ];
omc_error_34 = [ 1.639684e-03 ; 1.545762e-03 ; 3.139933e-03 ];
Tc_error_34  = [ 7.399380e-01 ; 7.489197e-01 ; 8.860480e-01 ];

%-- Image #35:
omc_35 = [ 3.053381e+00 ; -1.819574e-02 ; 3.823900e-02 ];
Tc_35  = [ -1.211865e+02 ; 1.138264e+02 ; 4.312969e+02 ];
omc_error_35 = [ 2.271773e-03 ; 5.383453e-04 ; 3.988013e-03 ];
Tc_error_35  = [ 7.870481e-01 ; 8.201887e-01 ; 9.984979e-01 ];

