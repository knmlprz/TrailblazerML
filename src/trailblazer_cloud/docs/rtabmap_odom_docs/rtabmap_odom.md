# RTAB-Map RGBD Odometry Parameters

This document lists the parameters for the `rgbd_odometry` node in RTAB-Map, as output by the command `ros2 run rtabmap_odom rgbd_odometry --params`. Each parameter is detailed with its value and description, organized into sections based on their functionality or module.

## Table of Contents

- [BRIEF Parameters](#brief-parameters)
- [BRISK Parameters](#brisk-parameters)
- [FAST Parameters](#fast-parameters)
- [FREAK Parameters](#freak-parameters)
- [GFTT Parameters](#gftt-parameters)
- [GMS Parameters](#gms-parameters)
- [GTSAM Parameters](#gtsam-parameters)
- [KAZE Parameters](#kaze-parameters)
- [Kp Parameters](#kp-parameters)
- [ORB Parameters](#orb-parameters)
- [Odom Parameters](#odom-parameters)
- [OdomF2M Parameters](#odomf2m-parameters)
- [OdomFovis Parameters](#odomfovis-parameters)
- [OdomLOAM Parameters](#odomloam-parameters)
- [OdomMSCKF Parameters](#odommsckf-parameters)
- [OdomMono Parameters](#odommono-parameters)
- [OdomOKVIS Parameters](#odomokvis-parameters)
- [OdomORBSLAM Parameters](#odomorbslam-parameters)
- [OdomOpen3D Parameters](#odomopen3d-parameters)
- [OdomOpenVINS Parameters](#odomopenvins-parameters)
- [OdomVINS Parameters](#odomvins-parameters)
- [OdomViso2 Parameters](#odomviso2-parameters)
- [Optimizer Parameters](#optimizer-parameters)
- [PyDetector Parameters](#pydetector-parameters)
- [PyMatcher Parameters](#pymatcher-parameters)
- [Reg Parameters](#reg-parameters)
- [Rtabmap Parameters](#rtabmap-parameters)
- [SIFT Parameters](#sift-parameters)
- [SURF Parameters](#surf-parameters)
- [SuperPoint Parameters](#superpoint-parameters)
- [Vis Parameters](#vis-parameters)
- [g2o Parameters](#g2o-parameters)
- [Warning](#warning)

## BRIEF Parameters

- **BRIEF/Bytes**: "32"  
  [Bytes is a length of descriptor in bytes. It can be equal 16, 32 or 64 bytes.]

## BRISK Parameters

- **BRISK/Octaves**: "3"  
  [Detection octaves. Use 0 to do single scale.]
- **BRISK/PatternScale**: "1"  
  [Apply this scale to the pattern used for sampling the neighbourhood of a keypoint.]
- **BRISK/Thresh**: "30"  
  [FAST/AGAST detection threshold score.]

## FAST Parameters

- **FAST/CV**: "0"  
  [Enable FastCV implementation if non-zero (and RTAB-Map is built with FastCV support). Values should be 9 and 10.]
- **FAST/Gpu**: "false"  
  [GPU-FAST: Use GPU version of FAST. This option is enabled only if OpenCV is built with CUDA and GPUs are detected.]
- **FAST/GpuKeypointsRatio**: "0.05"  
  [Used with FAST GPU.]
- **FAST/GridCols**: "0"  
  [Grid cols (0 to disable). Adapts the detector to partition the source image into a grid and detect points in each cell.]
- **FAST/GridRows**: "0"  
  [Grid rows (0 to disable). Adapts the detector to partition the source image into a grid and detect points in each cell.]
- **FAST/MaxThreshold**: "200"  
  [Maximum threshold. Used only when FAST/GridRows and FAST/GridCols are set.]
- **FAST/MinThreshold**: "7"  
  [Minimum threshold. Used only when FAST/GridRows and FAST/GridCols are set.]
- **FAST/NonmaxSuppression**: "true"  
  [If true, non-maximum suppression is applied to detected corners (keypoints).]
- **FAST/Threshold**: "20"  
  [Threshold on difference between intensity of the central pixel and pixels of a circle around this pixel.]

## FREAK Parameters

- **FREAK/NOctaves**: "4"  
  [Number of octaves covered by the detected keypoints.]
- **FREAK/OrientationNormalized**: "true"  
  [Enable orientation normalization.]
- **FREAK/PatternScale**: "22"  
  [Scaling of the description pattern.]
- **FREAK/ScaleNormalized**: "true"  
  [Enable scale normalization.]

## GFTT Parameters

- **GFTT/BlockSize**: "3"  
  []
- **GFTT/Gpu**: "false"  
  [GPU-GFTT: Use GPU version of GFTT. This option is enabled only if OpenCV>=3 is built with CUDA and GPUs are detected.]
- **GFTT/K**: "0.04"  
  []
- **GFTT/MinDistance**: "7"  
  []
- **GFTT/QualityLevel**: "0.001"  
  []
- **GFTT/UseHarrisDetector**: "false"  
  []

## GMS Parameters

- **GMS/ThresholdFactor**: "6.0"  
  [The higher, the less matches.]
- **GMS/WithRotation**: "false"  
  [Take rotation transformation into account.]
- **GMS/WithScale**: "false"  
  [Take scale transformation into account.]

## GTSAM Parameters

- **GTSAM/IncRelinearizeSkip**: "1"  
  [Only relinearize any variables every X calls to ISAM2::update(). See GTSAM::ISAM2 doc for more info.]
- **GTSAM/IncRelinearizeThreshold**: "0.01"  
  [Only relinearize variables whose linear delta magnitude is greater than this threshold. See GTSAM::ISAM2 doc for more info.]
- **GTSAM/Incremental**: "false"  
  [Do graph optimization incrementally (iSAM2) to increase optimization speed on loop closures. Note that only GaussNewton and Dogleg optimization algorithms are supported (GTSAM/Optimizer) in this mode.]
- **GTSAM/Optimizer**: "1"  
  [0=Levenberg 1=GaussNewton 2=Dogleg]

## KAZE Parameters

- **KAZE/Diffusivity**: "1"  
  [Diffusivity type: 0=DIFF_PM_G1, 1=DIFF_PM_G2, 2=DIFF_WEICKERT or 3=DIFF_CHARBONNIER.]
- **KAZE/Extended**: "false"  
  [Set to enable extraction of extended (128-byte) descriptor.]
- **KAZE/NOctaveLayers**: "4"  
  [Default number of sublevels per scale level.]
- **KAZE/NOctaves**: "4"  
  [Maximum octave evolution of the image.]
- **KAZE/Threshold**: "0.001"  
  [Detector response threshold to accept keypoint.]
- **KAZE/Upright**: "false"  
  [Set to enable use of upright descriptors (non rotation-invariant).]

## Kp Parameters

- **Kp/ByteToFloat**: "false"  
  [For Kp/NNStrategy=1, binary descriptors are converted to float by converting each byte to float instead of converting each bit to float. When converting bytes instead of bits, less memory is used and search is faster at the cost of slightly less accurate matching.]

## ORB Parameters

- **ORB/EdgeThreshold**: "19"  
  [This is size of the border where the features are not detected. It should roughly match the patchSize parameter.]
- **ORB/FirstLevel**: "0"  
  [It should be 0 in the current implementation.]
- **ORB/Gpu**: "false"  
  [GPU-ORB: Use GPU version of ORB. This option is enabled only if OpenCV is built with CUDA and GPUs are detected.]
- **ORB/NLevels**: "3"  
  [The number of pyramid levels. The smallest level will have linear size equal to input_image_linear_size/pow(scaleFactor, nlevels).]
- **ORB/PatchSize**: "31"  
  [size of the patch used by the oriented BRIEF descriptor. Of course, on smaller pyramid layers the perceived image area covered by a feature will be larger.]
- **ORB/ScaleFactor**: "2"  
  [Pyramid decimation ratio, greater than 1. scaleFactor==2 means the classical pyramid, where each next level has 4x less pixels than the previous, but such a big scale factor will degrade feature matching scores dramatically. On the other hand, too close to 1 scale factor will mean that to cover certain scale range you will need more pyramid levels and so the speed will suffer.]
- **ORB/ScoreType**: "0"  
  [The default HARRIS_SCORE=0 means that Harris algorithm is used to rank features (the score is written to KeyPoint::score and is used to retain best nfeatures features); FAST_SCORE=1 is alternative value of the parameter that produces slightly less stable keypoints, but it is a little faster to compute.]
- **ORB/WTA_K**: "2"  
  [The number of points that produce each element of the oriented BRIEF descriptor. The default value 2 means the BRIEF where we take a random point pair and compare their brightnesses, so we get 0/1 response. Other possible values are 3 and 4. For example, 3 means that we take 3 random points (of course, those point coordinates are random, but they are generated from the pre-defined seed, so each element of BRIEF descriptor is computed deterministically from the pixel rectangle), find point of maximum brightness and output index of the winner (0, 1 or 2). Such output will occupy 2 bits, and therefore it will need a special variant of Hamming distance, denoted as NORM_HAMMING2 (2 bits per bin). When WTA_K=4, we take 4 random points to compute each bin (that will also occupy 2 bits with possible values 0, 1, 2 or 3).]

## Odom Parameters

- **Odom/AlignWithGround**: "false"  
  [Align odometry with the ground on initialization.]
- **Odom/Deskewing**: "true"  
  [Lidar deskewing. If input lidar has time channel, it will be deskewed with a constant motion model (with IMU orientation and/or guess if provided).]
- **Odom/FillInfoData**: "true"  
  [Fill info with data (inliers/outliers features).]
- **Odom/FilteringStrategy**: "0"  
  [0=No filtering 1=Kalman filtering 2=Particle filtering. This filter is used to smooth the odometry output.]
- **Odom/GuessMotion**: "true"  
  [Guess next transformation from the last motion computed.]
- **Odom/GuessSmoothingDelay**: "0"  
  [Guess smoothing delay (s). Estimated velocity is averaged based on last transforms up to this maximum delay. This can help to get smoother velocity prediction. Last velocity computed is used directly if "Odom/FilteringStrategy" is set or the delay is below the odometry rate.]
- **Odom/Holonomic**: "true"  
  [If the robot is holonomic (strafing commands can be issued). If not, y value will be estimated from x and yaw values (y=x*tan(yaw)).]
- **Odom/ImageBufferSize**: "1"  
  [Data buffer size (0 min inf).]
- **Odom/ImageDecimation**: "1"  
  [Decimation of the RGB image before registration. If depth size is larger than decimated RGB size, depth is decimated to be always at most equal to RGB size. If Vis/DepthAsMask is true and if depth is smaller than decimated RGB, depth may be interpolated to match RGB size for feature detection.]
- **Odom/KalmanMeasurementNoise**: "0.01"  
  [Process measurement covariance value.]
- **Odom/KalmanProcessNoise**: "0.001"  
  [Process noise covariance value.]
- **Odom/KeyFrameThr**: "0.3"  
  [[Visual] Create a new keyframe when the number of inliers drops under this ratio of features in last frame. Setting the value to 0 means that a keyframe is created for each processed frame.]
- **Odom/ParticleLambdaR**: "100"  
  [Lambda of rotational components (roll,pitch,yaw).]
- **Odom/ParticleLambdaT**: "100"  
  [Lambda of translation components (x,y,z).]
- **Odom/ParticleNoiseR**: "0.002"  
  [Noise (rad) of rotational components (roll,pitch,yaw).]
- **Odom/ParticleNoiseT**: "0.002"  
  [Noise (m) of translation components (x,y,z).]
- **Odom/ParticleSize**: "400"  
  [Number of particles of the filter.]
- **Odom/ResetCountdown**: "0"  
  [Automatically reset odometry after X consecutive images on which odometry cannot be computed (value=0 disables auto-reset).]
- **Odom/ScanKeyFrameThr**: "0.9"  
  [[Geometry] Create a new keyframe when the number of ICP inliers drops under this ratio of points in last frame's scan. Setting the value to 0 means that a keyframe is created for each processed frame.]
- **Odom/Strategy**: "0"  
  [0=Frame-to-Map (F2M) 1=Frame-to-Frame (F2F) 2=Fovis 3=viso2 4=DVO-SLAM 5=ORB_SLAM2 6=OKVIS 7=LOAM 8=MSCKF_VIO 9=VINS-Fusion 10=OpenVINS 11=FLOAM 12=Open3D]
- **Odom/VisKeyFrameThr**: "150"  
  [[Visual]JOHN Create a new keyframe when the number of inliers drops under this threshold. Setting the value to 0 means that a keyframe is created for each processed frame.]

## OdomF2M Parameters

- **OdomF2M/BundleAdjustment**: "1"  
  [Local bundle adjustment: 0=disabled, 1=g2o, 2=cvsba, 3=Ceres.]
- **OdomF2M/BundleAdjustmentMaxFrames**: "10"  
  [Maximum frames used for bundle adjustment (0=inf or all current frames in the local map).]
- **OdomF2M/MaxNewFeatures**: "0"  
  [[Visual] Maximum features (sorted by keypoint response) added to local map from a new key-frame. 0 means no limit.]
- **OdomF2M/MaxSize**: "2000"  
  [[Visual] Local map size: If > 0 (example 5000), the odometry will maintain a local map of X maximum words.]
- **OdomF2M/ScanMaxSize**: "2000"  
  [[Geometry] Maximum local scan map size.]
- **OdomF2M/ScanRange**: "0"  
  [[Geometry] Distance Range used to filter points of local map (when > 0). 0 means local map is updated using time and not range.]
- **OdomF2M/ScanSubtractAngle**: "45"  
  [[Geometry] Max angle (degrees) used to filter points of a new added scan to local map (when "OdomF2M/ScanSubtractRadius">0). 0 means any angle.]
- **OdomF2M/ScanSubtractRadius**: "0.05"  
  [[Geometry] Radius used to filter points of a new added scan to local map. This could match the voxel size of the scans.]
- **OdomF2M/ValidDepthRatio**: "0.75"  
  [If a new frame has points without valid depth, they are added to local feature map only if points with valid depth on total points is over this ratio. Setting to 1 means no points without valid depth are added to local feature map.]

## OdomFovis Parameters

- **OdomFovis/BucketHeight**: "80"  
  []
- **OdomFovis/BucketWidth**: "80"  
  []
- **OdomFovis/CliqueInlierThreshold**: "0.1"  
  [See Howard's greedy max-clique algorithm for determining the maximum set of mutually consistent feature matches. This specifies the compatibility threshold, in meters.]
- **OdomFovis/FastThreshold**: "20"  
  [FAST threshold.]
- **OdomFovis/FastThresholdAdaptiveGain**: "0.005"  
  [FAST threshold adaptive gain.]
- **OdomFovis/FeatureSearchWindow**: "25"  
  [Specifies the size of the search window to apply when searching for feature matches across time frames. The search is conducted around the feature location predicted by the initial rotation estimate.]
- **OdomFovis/FeatureWindowSize**: "9"  
  [The size of the n x n image patch surrounding each feature, used for keypoint matching.]
- **OdomFovis/InlierMaxReprojectionError**: "1.5"  
  [The maximum image-space reprojection error (in pixels) a feature match is allowed to have and still be considered an inlier in the set of features used for motion estimation.]
- **OdomFovis/MaxKeypointsPerBucket**: "25"  
  []
- **OdomFovis/MaxMeanReprojectionError**: "10.0"  
  [Maximum mean reprojection error over the inlier feature matches for the motion estimate to be considered valid.]
- **OdomFovis/MaxPyramidLevel**: "3"  
  [The maximum Gaussian pyramid level to process the image at. Pyramid level 1 corresponds to the original image.]
- **OdomFovis/MinFeaturesForEstimate**: "20"  
  [Minimum number of features in the inlier set for the motion estimate to be considered valid.]
- **OdomFovis/MinPyramidLevel**: "0"  
  [The minimum pyramid level.]
- **OdomFovis/StereoMaxDisparity**: "128"  
  []
- **OdomFovis/StereoMaxDistEpipolarLine**: "1.5"  
  []
- **OdomFovis/StereoMaxRefinementDisplacement**: "1.0"  
  []
- **OdomFovis/StereoRequireMutualMatch**: "true"  
  []
- **OdomFovis/TargetPixelsPerFeature**: "250"  
  [Specifies the desired feature density as a ratio of input image pixels per feature detected. This number is used to control the adaptive feature thresholding.]
- **OdomFovis/UpdateTargetFeaturesWithRefined**: "false"  
  [When subpixel refinement is enabled, the refined feature locations can be saved over the original feature locations. This has a slightly negative impact on frame-to-frame visual odometry, but is likely better when using this library as part of a visual SLAM algorithm.]
- **OdomFovis/UseAdaptiveThreshold**: "true"  
  [Use FAST adaptive threshold.]
- **OdomFovis/UseBucketing**: "true"  
  []
- **OdomFovis/UseHomographyInitialization**: "true"  
  [Use homography initialization.]
- **OdomFovis/UseImageNormalization**: "false"  
  []
- **OdomFovis/UseSubpixelRefinement**: "true"  
  [Specifies whether or not to refine feature matches to subpixel resolution.]

## OdomLOAM Parameters

- **OdomLOAM/AngVar**: "0.01"  
  [Angular output variance.]
- **OdomLOAM/LinVar**: "0.01"  
  [Linear output variance.]
- **OdomLOAM/LocalMapping**: "true"  
  [Local mapping. It adds more time to compute odometry, but accuracy is significantly improved.]
- **OdomLOAM/Resolution**: "0.2"  
  [Map resolution]
- **OdomLOAM/ScanPeriod**: "0.1"  
  [Scan period (s)]
- **OdomLOAM/Sensor**: "2"  
  [Velodyne sensor: 0=VLP-16, 1=HDL-32, 2=HDL-64E]

## OdomMSCKF Parameters

- **OdomMSCKF/FastThreshold**: "10"  
  []
- **OdomMSCKF/GridCol**: "5"  
  []
- **OdomMSCKF/GridMaxFeatureNum**: "4"  
  []
- **OdomMSCKF/GridMinFeatureNum**: "3"  
  []
- **OdomMSCKF/GridRow**: "4"  
  []
- **OdomMSCKF/InitCovAccBias**: "0.01"  
  []
- **OdomMSCKF/InitCovExRot**: "0.00030462"  
  []
- **OdomMSCKF/InitCovExTrans**: "0.000025"  
  []
- **OdomMSCKF/InitCovGyroBias**: "0.01"  
  []
- **OdomMSCKF/InitCovVel**: "0.25"  
  []
- **OdomMSCKF/MaxCamStateSize**: "20"  
  []
- **OdomMSCKF/MaxIteration**: "30"  
  []
- **OdomMSCKF/NoiseAcc**: "0.05"  
  []
- **OdomMSCKF/NoiseAccBias**: "0.01"  
  []
- **OdomMSCKF/NoiseFeature**: "0.035"  
  []
- **OdomMSCKF/NoiseGyro**: "0.005"  
  []
- **OdomMSCKF/NoiseGyroBias**: "0.001"  
  []
- **OdomMSCKF/OptTranslationThreshold**: "0"  
  []
- **OdomMSCKF/PatchSize**: "15"  
  []
- **OdomMSCKF/PositionStdThreshold**: "8.0"  
  []
- **OdomMSCKF/PyramidLevels**: "3"  
  []
- **OdomMSCKF/RansacThreshold**: "3"  
  []
- **OdomMSCKF/RotationThreshold**: "0.2618"  
  []
- **OdomMSCKF/StereoThreshold**: "5"  
  []
- **OdomMSCKF/TrackPrecision**: "0.01"  
  []
- **OdomMSCKF/TrackingRateThreshold**: "0.5"  
  []
- **OdomMSCKF/TranslationThreshold**: "0.4"  
  []

## OdomMono Parameters

- **OdomMono/InitMinFlow**: "100"  
  [Minimum optical flow required for the initialization step.]
- **OdomMono/InitMinTranslation**: "0.1"  
  [Minimum translation required for the initialization step.]
- **OdomMono/MaxVariance**: "0.01"  
  [Maximum variance to add new points to local map.]
- **OdomMono/MinTranslation**: "0.02"  
  [Minimum translation to add new points to local map. On initialization, translation x 5 is used as the minimum.]

## OdomOKVIS Parameters

- **OdomOKVIS/ConfigPath**: ""  
  [Path of OKVIS config file.]

## OdomORBSLAM Parameters

- **OdomORBSLAM/AccNoise**: "0.1"  
  [IMU accelerometer "white noise".]
- **OdomORBSLAM/AccWalk**: "0.0001"  
  [IMU accelerometer "random walk".]
- **OdomORBSLAM/Bf**: "0.076"  
  [Fake IR projector baseline (m) used only when stereo is not used.]
- **OdomORBSLAM/Fps**: "0.0"  
  [Camera FPS (0 to estimate from input data).]
- **OdomORBSLAM/GyroNoise**: "0.01"  
  [IMU gyroscope "white noise".]
- **OdomORBSLAM/GyroWalk**: "0.000001"  
  [IMU gyroscope "random walk".]
- **OdomORBSLAM/Inertial**: "false"  
  [Enable IMU. Only supported with ORB_SLAM3.]
- **OdomORBSLAM/MapSize**: "3000"  
  [Maximum size of the feature map (0 means infinite). Only supported with ORB_SLAM2.]
- **OdomORBSLAM/MaxFeatures**: "1000"  
  [Maximum ORB features extracted per frame.]
- **OdomORBSLAM/SamplingRate**: "0"  
  [IMU sampling rate (0 to estimate from input data).]
- **OdomORBSLAM/ThDepth**: "40.0"  
  [Close/Far threshold. Baseline times.]
- **OdomORBSLAM/VocPath**: ""  
  [Path to ORB vocabulary (*.txt).]

## OdomOpen3D Parameters

- **OdomOpen3D/MaxDepth**: "3.0"  
  [Maximum depth.]
- **OdomOpen3D/Method**: "0"  
  [Registration method: 0=PointToPlane, 1=Intensity, 2=Hybrid.]

## OdomOpenVINS Parameters

- **OdomOpenVINS/AccelerometerNoiseDensity**: "0.01"  
  [[m/s^2/sqrt(Hz)] (accel "white noise")]
- **OdomOpenVINS/AccelerometerRandomWalk**: "0.001"  
  [[m/s^3/sqrt(Hz)] (accel bias diffusion)]
- **OdomOpenVINS/CalibCamExtrinsics**: "false"  
  [Bool to determine whether or not to calibrate imu-to-camera pose]
- **OdomOpenVINS/CalibCamIntrinsics**: "false"  
  [Bool to determine whether or not to calibrate camera intrinsics]
- **OdomOpenVINS/CalibCamTimeoffset**: "false"  
  [Bool to determine whether or not to calibrate camera to IMU time offset]
- **OdomOpenVINS/CalibIMUGSensitivity**: "false"  
  [Bool to determine whether or not to calibrate the Gravity sensitivity]
- **OdomOpenVINS/CalibIMUIntrinsics**: "false"  
  [Bool to determine whether or not to calibrate the IMU intrinsics]
- **OdomOpenVINS/DtSLAMDelay**: "0.0"  
  [Delay, in seconds, that we should wait from init before we start estimating SLAM features]
- **OdomOpenVINS/FeatRepMSCKF**: "0"  
  [What representation our features are in (msckf features)]
- **OdomOpenVINS/FeatRepSLAM**: "4"  
  [What representation our features are in (slam features)]
- **OdomOpenVINS/FiMaxBaseline**: "40"  
  [Max baseline ratio to accept triangulated features]
- **OdomOpenVINS/FiMaxCondNumber**: "10000"  
  [Max condition number of linear triangulation matrix accept triangulated features]
- **OdomOpenVINS/FiMaxRuns**: "5"  
  [Max runs for Levenberg-Marquardt]
- **OdomOpenVINS/FiRefineFeatures**: "true"  
  [If we should perform Levenberg-Marquardt refinement]
- **OdomOpenVINS/FiTriangulate1d**: "false"  
  [If we should perform 1d triangulation instead of 3d]
- **OdomOpenVINS/GravityMag**: "9.81"  
  [Gravity magnitude in the global frame (i.e. should be 9.81 typically)]
- **OdomOpenVINS/GyroscopeNoiseDensity**: "0.001"  
  [[rad/s/sqrt(Hz)] (gyro "white noise")]
- **OdomOpenVINS/GyroscopeRandomWalk**: "0.0001"  
  [[rad/s^2/sqrt(Hz)] (gyro bias diffusion)]
- **OdomOpenVINS/InitDynInflationBa**: "100.0"  
  [What to inflate the recovered bias_a covariance by]
- **OdomOpenVINS/InitDynInflationBg**: "10.0"  
  [What to inflate the recovered bias_g covariance by]
- **OdomOpenVINS/InitDynInflationOri**: "10.0"  
  [What to inflate the recovered q_GtoI covariance by]
- **OdomOpenVINS/InitDynInflationVel**: "100.0"  
  [What to inflate the recovered v_IinG covariance by]
- **OdomOpenVINS/InitDynMLEMaxIter**: "50"  
  [How many iterations the MLE refinement should use (zero to skip the MLE)]
- **OdomOpenVINS/InitDynMLEMaxThreads**: "6"  
  [How many threads the MLE should use]
- **OdomOpenVINS/InitDynMLEMaxTime**: "0.05"  
  [How many seconds the MLE should be completed in]
- **OdomOpenVINS/InitDynMLEOptCalib**: "false"  
  [If we should optimize calibration during intialization (not recommended)]
- **OdomOpenVINS/InitDynMinDeg**: "10.0"  
  [Orientation change needed to try to init]
- **OdomOpenVINS/InitDynMinRecCond**: "1e-15"  
  [Reciprocal condition number thresh for info inversion]
- **OdomOpenVINS/InitDynNumPose**: "6"  
  [Number of poses to use within our window time (evenly spaced)]
- **OdomOpenVINS/InitDynUse**: "false"  
  [If dynamic initialization should be used]
- **OdomOpenVINS/InitIMUThresh**: "1.0"  
  [Variance threshold on our acceleration to be classified as moving]
- **OdomOpenVINS/InitMaxDisparity**: "10.0"  
  [Max disparity to consider the platform stationary (dependent on resolution)]
- **OdomOpenVINS/InitMaxFeatures**: "50"  
  [How many features to track during initialization (saves on computation)]
- **OdomOpenVINS/InitWindowTime**: "2.0"  
  [Amount of time we will initialize over (seconds)]
- **OdomOpenVINS/Integration**: "1"  
  [0=discrete, 1=rk4, 2=analytical (if rk4 or analytical used then analytical covariance propagation is used)]
- **OdomOpenVINS/LeftMaskPath**: ""  
  [Mask for left image]
- **OdomOpenVINS/MaxClones**: "11"  
  [Max clone size of sliding window]
- **OdomOpenVINS/MaxMSCKFInUpdate**: "50"  
  [Max number of MSCKF features we will use at a given image timestep.]
- **OdomOpenVINS/MaxSLAM**: "50"  
  [Max number of estimated SLAM features]
- **OdomOpenVINS/MaxSLAMInUpdate**: "25"  
  [Max number of SLAM features we allow to be included in a single EKF update.]
- **OdomOpenVINS/MinPxDist**: "15"  
  [Distance between features (features near each other provide less information)]
- **OdomOpenVINS/NumPts**: "200"  
  [Number of points (per camera) we will extract and try to track]
- **OdomOpenVINS/RightMaskPath**: ""  
  [Mask for right image]
- **OdomOpenVINS/TryZUPT**: "true"  
  [If we should try to use zero velocity update]
- **OdomOpenVINS/UpMSCKFChi2Multiplier**: "1.0"  
  [Chi2 multiplier for MSCKF features]
- **OdomOpenVINS/UpMSCKFSigmaPx**: "1.0"  
  [Pixel noise for MSCKF features]
- **OdomOpenVINS/UpSLAMChi2Multiplier**: "1.0"  
  [Chi2 multiplier for SLAM features]
- **OdomOpenVINS/UpSLAMSigmaPx**: "1.0"  
  [Pixel noise for SLAM features]
- **OdomOpenVINS/UseFEJ**: "true"  
  [If first-estimate Jacobians should be used (enable for good consistency)]
- **OdomOpenVINS/UseKLT**: "true"  
  [If true we will use KLT, otherwise use a ORB descriptor + robust matching]
- **OdomOpenVINS/UseStereo**: "true"  
  [If we have more than 1 camera, if we should try to track stereo constraints between pairs]
- **OdomOpenVINS/ZUPTChi2Multiplier**: "0.0"  
  [Chi2 multiplier for zero velocity]
- **OdomOpenVINS/ZUPTMaxDisparity**: "0.5"  
  [Max disparity we will consider to try to do a zupt (i.e. if above this, don't do zupt)]
- **OdomOpenVINS/ZUPTMaxVelocity**: "0.1"  
  [Max velocity we will consider to try to do a zupt (i.e. if above this, don't do zupt)]
- **OdomOpenVINS/ZUPTNoiseMultiplier**: "10.0"  
  [Multiplier of our zupt measurement IMU noise matrix (default should be 1.0)]
- **OdomOpenVINS/ZUPTOnlyAtBeginning**: "false"  
  [If we should only use the zupt at the very beginning static initialization phase]

## OdomVINS Parameters

- **OdomVINS/ConfigPath**: ""  
  [Path of VINS config file.]

## OdomViso2 Parameters

- **OdomViso2/BucketHeight**: "50"  
  [Height of bucket.]
- **OdomViso2/BucketMaxFeatures**: "2"  
  [Maximal number of features per bucket.]
- **OdomViso2/BucketWidth**: "50"  
  [Width of bucket.]
- **OdomViso2/InlierThreshold**: "2.0"  
  [Fundamental matrix inlier threshold.]
- **OdomViso2/MatchBinsize**: "50"  
  [Matching bin width/height (affects efficiency only).]
- **OdomViso2/MatchDispTolerance**: "2"  
  [Disparity tolerance for stereo matches (in pixels).]
- **OdomViso2/MatchHalfResolution**: "true"  
  [Match at half resolution, refine at full resolution.]
- **OdomViso2/MatchMultiStage**: "true"  
  [Multistage matching (denser and faster).]
- **OdomViso2/MatchNmsN**: "3"  
  [Non-max-suppression: min. distance between maxima (in pixels).]
- **OdomViso2/MatchNmsTau**: "50"  
  [Non-max-suppression: interest point peakiness threshold.]
- **OdomViso2/MatchOutlierDispTolerance**: "5"  
  [Outlier removal: disparity tolerance (in pixels).]
- **OdomViso2/MatchOutlierFlowTolerance**: "5"  
  [Outlier removal: flow tolerance (in pixels).]
- **OdomViso2/MatchRadius**: "200"  
  [Matching radius (du/dv in pixels).]
- **OdomViso2/MatchRefinement**: "1"  
  [Refinement (0=none,1=pixel,2=subpixel).]
- **OdomViso2/RansacIters**: "200"  
  [Number of RANSAC iterations.]
- **OdomViso2/Reweighting**: "true"  
  [Lower border weights (more robust to calibration errors).]

## Optimizer Parameters

- **Optimizer/Epsilon**: "0.00001"  
  [Stop optimizing when the error improvement is less than this value.]
- **Optimizer/GravitySigma**: "0.3"  
  [Gravity sigma value (>=0, typically between 0.1 and 0.3). Optimization is done while preserving gravity orientation of the poses. This should be used only with visual/lidar inertial odometry approaches, for which we assume that all odometry poses are aligned with gravity. Set to 0 to disable gravity constraints. Currently supported only with g2o and GTSAM optimization strategies (see Optimizer/Strategy).]
- **Optimizer/Iterations**: "20"  
  [Optimization iterations.]
- **Optimizer/LandmarksIgnored**: "false"  
  [Ignore landmark constraints while optimizing. Currently only g2o and gtsam optimization supports this.]
- **Optimizer/PriorsIgnored**: "true"  
  [Ignore prior constraints (global pose or GPS) while optimizing. Currently only g2o and gtsam optimization supports this.]
- **Optimizer/Robust**: "false"  
  [Robust graph optimization using Vertigo (only work for g2o and GTSAM optimization strategies). Not compatible with "RGBD/OptimizeMaxError" if enabled.]
- **Optimizer/Strategy**: "2"  
  [Graph optimization strategy: 0=TORO, 1=g2o, 2=GTSAM and 3=Ceres.]
- **Optimizer/VarianceIgnored**: "false"  
  [Ignore constraints' variance. If checked, identity information matrix is used for each constraint. Otherwise, an information matrix is generated from the variance saved in the links.]

## PyDetector Parameters

- **PyDetector/Cuda**: "true"  
  [Use cuda.]
- **PyDetector/Path**: ""  
  [Path to python script file (see available ones in rtabmap/corelib/src/python/*). See the header to see where the script should be copied.]

## PyMatcher Parameters

- **PyMatcher/Cuda**: "true"  
  [Used by SuperGlue.]
- **PyMatcher/Iterations**: "20"  
  [Sinkhorn iterations. Used by SuperGlue.]
- **PyMatcher/Model**: "indoor"  
  [For SuperGlue, set only "indoor" or "outdoor". For OANet, set path to one of the pth file (e.g., "OANet/model/gl3d/sift-4000/model_best.pth").]
- **PyMatcher/Path**: ""  
  [Path to python script file (see available ones in rtabmap/corelib/src/python/*). See the header to see where the script should be copied.]
- **PyMatcher/Threshold**: "0.2"  
  [Used by SuperGlue.]

## Reg Parameters

- **Reg/Force3DoF**: "false"  
  [Force 3 degrees-of-freedom transform (3Dof: x,y and yaw). Parameters z, roll and pitch will be set to 0.]
- **Reg/RepeatOnce**: "true"  
  [Do a second registration with the output of the first registration as guess. Only done if no guess was provided for the first registration (like on loop closure). It can be useful if the registration approach used can use a guess to get better matches.]
- **Reg/Strategy**: "0"  
  [0=Vis, 1=Icp, 2=VisIcp]

## Rtabmap Parameters

- **Rtabmap/ImagesAlreadyRectified**: "true"  
  [Images are already rectified. By default RTAB-Map assumes that received images are rectified. If they are not, they can be rectified by RTAB-Map if this parameter is false.]
- **Rtabmap/PublishRAMUsage**: "false"  
  [Publishing RAM usage in statistics (may add a small overhead to get info from the system).]

## SIFT Parameters

- **SIFT/ContrastThreshold**: "0.04"  
  [The contrast threshold used to filter out weak features in semi-uniform (low-contrast) regions. The larger the threshold, the less features are produced by the detector. Not used by CudaSift (see SIFT/GaussianThreshold instead).]
- **SIFT/EdgeThreshold**: "10"  
  [The threshold used to filter out edge-like features. Note that the its meaning is different from the contrastThreshold, i.e. the larger the edgeThreshold, the less features are filtered out (more features are retained).]
- **SIFT/GaussianThreshold**: "2.0"  
  [CudaSift: Threshold on difference of Gaussians for feature pruning. The higher the threshold, the less features are produced by the detector.]
- **SIFT/Gpu**: "false"  
  [CudaSift: Use GPU version of SIFT. This option is enabled only if RTAB-Map is built with CudaSift dependency and GPUs are detected.]
- **SIFT/NOctaveLayers**: "3"  
  [The number of layers in each octave. 3 is the value used in D. Lowe paper. The number of octaves is computed automatically from the image resolution. Not used by CudaSift, the number of octaves is still computed automatically.]
- **SIFT/PreciseUpscale**: "false"  
  [Whether to enable precise upscaling in the scale pyramid (OpenCV >= 4.8).]
- **SIFT/RootSIFT**: "false"  
  [Apply RootSIFT normalization of the descriptors.]
- **SIFT/Sigma**: "1.6"  
  [The sigma of the Gaussian applied to the input image at the octave #0. If your image is captured with a weak camera with soft lenses, you might want to reduce the number.]
- **SIFT/Upscale**: "false"  
  [CudaSift: Whether to enable upscaling.]

## SURF Parameters

- **SURF/Extended**: "false"  
  [Extended descriptor flag (true - use extended 128-element descriptors; false - use 64-element descriptors).]
- **SURF/GpuKeypointsRatio**: "0.01"  
  [Used with SURF GPU.]
- **SURF/GpuVersion**: "false"  
  [GPU-SURF: Use GPU version of SURF. This option is enabled only if OpenCV is built with CUDA and GPUs are detected.]
- **SURF/HessianThreshold**: "500"  
  [Threshold for hessian keypoint detector used in SURF.]
- **SURF/OctaveLayers**: "2"  
  [Number of octave layers within each octave.]
- **SURF/Octaves**: "4"  
  [Number of pyramid octaves the keypoint detector will use.]
- **SURF/Upright**: "false"  
  [Up-right or rotated features flag (true - do not compute orientation of features; false - compute orientation).]

## SuperPoint Parameters

- **SuperPoint/Cuda**: "true"  
  [Use Cuda device for Torch, otherwise CPU device is used by default.]
- **SuperPoint/ModelPath**: ""  
  [[Required] Path to pre-trained weights Torch file of SuperPoint (*.pt).]
- **SuperPoint/NMS**: "true"  
  [If true, non-maximum suppression is applied to detected keypoints.]
- **SuperPoint/NMSRadius**: "4"  
  [[SuperPoint/NMS=true] Minimum distance (pixels) between keypoints.]
- **SuperPoint/Threshold**: "0.010"  
  [Detector response threshold to accept keypoint.]

## Vis Parameters

- **Vis/BundleAdjustment**: "1"  
  [Optimization with bundle adjustment: 0=disabled, 1=g2o, 2=cvsba, 3=Ceres.]
- **Vis/CorFlowEps**: "0.01"  
  [[Vis/CorType=1] See cv::calcOpticalFlowPyrLK(). Used for optical flow approach.]
- **Vis/CorFlowGpu**: "false"  
  [[Vis/CorType=1] Enable GPU version of the optical flow approach (only available if OpenCV is built with CUDA).]
- **Vis/CorFlowIterations**: "30"  
  [[Vis/CorType=1] See cv::calcOpticalFlowPyrLK(). Used for optical flow approach.]
- **Vis/CorFlowMaxLevel**: "3"  
  [[Vis/CorType=1] See cv::calcOpticalFlowPyrLK(). Used for optical flow approach.]
- **Vis/CorFlowWinSize**: "16"  
  [[Vis/CorType=1] See cv::calcOpticalFlowPyrLK(). Used for optical flow approach.]
- **Vis/CorGuessMatchToProjection**: "false"  
  [[Vis/CorType=0] Match frame's corners to source's projected points (when guess transform is provided) instead of projected points to frame's corners.]
- **Vis/CorGuessWinSize**: "40"  
  [[Vis/CorType=0] Matching window size (pixels) around projected points when a guess transform is provided to find correspondences. 0 means disabled.]
- **Vis/CorNNDR**: "0.8"  
  [[Vis/CorType=0] NNDR: nearest neighbor distance ratio. Used for knn features matching approach.]
- **Vis/CorNNType**: "1"  
  [[Vis/CorType=0] kNNFlannNaive=0, kNNFlannKdTree=1, kNNFlannLSH=2, kNNBruteForce=3, kNNBruteForceGPU=4, BruteForceCrossCheck=5, SuperGlue=6, GMS=7. Used for features matching approach.]
- **Vis/CorType**: "0"  
  [Correspondences computation approach: 0=Features Matching, 1=Optical Flow]
- **Vis/DepthAsMask**: "true"  
  [Use depth image as mask when extracting features.]
- **Vis/DepthMaskFloorThr**: "0.0"  
  [Filter floor from depth mask below specified threshold (m) before extracting features. 0 means disabled, negative means remove all objects above the floor threshold instead. Ignored if Vis/DepthAsMask is false.]
- **Vis/EpipolarGeometryVar**: "0.1"  
  [[Vis/EstimationType = 2] Epipolar geometry maximum variance to accept the transformation.]
- **Vis/EstimationType**: "1"  
  [Motion estimation approach: 0:3D->3D, 1:3D->2D (PnP), 2:2D->2D (Epipolar Geometry)]
- **Vis/FeatureType**: "8"  
  [0=SURF 1=SIFT 2=ORB 3=FAST/FREAK 4=FAST/BRIEF 5=GFTT/FREAK 6=GFTT/BRIEF 7=BRISK 8=GFTT/ORB 9=KAZE 10=ORB-OCTREE 11=SuperPoint 12=SURF/FREAK 13=GFTT/DAISY 14=SURF/DAISY 15=PyDetector]
- **Vis/ForwardEstOnly**: "true"  
  [Forward estimation only (A->B). If false, a transformation is also computed in backward direction (B->A), then the two resulting transforms are merged (middle interpolation between the transforms).]
- **Vis/GridCols**: "1"  
  [Number of columns of the grid used to extract uniformly "Vis/MaxFeatures / grid cells" features from each cell.]
- **Vis/GridRows**: "1"  
  [Number of rows of the grid used to extract uniformly "Vis/MaxFeatures / grid cells" features from each cell.]
- **Vis/InlierDistance**: "0.1"  
  [[Vis/EstimationType = 0] Maximum distance for feature correspondences. Used by 3D->3D estimation approach.]
- **Vis/Iterations**: "300"  
  [Maximum iterations to compute the transform.]
- **Vis/MaxDepth**: "0"  
  [Max depth of the features (0 means no limit).]
- **Vis/MaxFeatures**: "1000"  
  [0 no limits.]
- **Vis/MeanInliersDistance**: "0.0"  
  [Maximum distance (m) of the mean distance of inliers from the camera to accept the transformation. 0 means disabled.]
- **Vis/MinDepth**: "0"  
  [Min depth of the features (0 means no limit).]
- **Vis/MinInliers**: "20"  
  [Minimum feature correspondences to compute/accept the transformation.]
- **Vis/MinInliersDistribution**: "0.0"  
  [Minimum distribution value of the inliers in the image to accept the transformation. The distribution is the second eigen value of the PCA (Principal Component Analysis) on the keypoints of the normalized image [-0.5, 0.5]. The value would be between 0 and 0.5. 0 means disabled.]
- **Vis/PnPFlags**: "0"  
  [[Vis/EstimationType = 1] PnP flags: 0=Iterative, 1=EPNP, 2=P3P]
- **Vis/PnPMaxVariance**: "0.0"  
  [[Vis/EstimationType = 1] Max linear variance between 3D point correspondences after PnP. 0 means disabled.]
- **Vis/PnPRefineIterations**: "0"  
  [[Vis/EstimationType = 1] Refine iterations. Set to 0 if "Vis/BundleAdjustment" is also used.]
- **Vis/PnPReprojError**: "2"  
  [[Vis/EstimationType = 1] PnP reprojection error.]
- **Vis/PnPSamplingPolicy**: "1"  
  [[Vis/EstimationType = 1] Multi-camera random sampling policy: 0=AUTO, 1=ANY, 2=HOMOGENEOUS. With HOMOGENEOUS policy, RANSAC will be done uniformly against all cameras, so at least 2 matches per camera are required. With ANY policy, RANSAC is not constraint to sample on all cameras at the same time. AUTO policy will use HOMOGENEOUS if there are at least 2 matches per camera, otherwise it will fallback to ANY policy.]
- **Vis/PnPSplitLinearCovComponents**: "false"  
  [[Vis/EstimationType = 1] Compute variance for each linear component instead of using the combined XYZ variance for all linear components.]
- **Vis/PnPVarianceMedianRatio**: "4"  
  [[Vis/EstimationType = 1] Ratio used to compute variance of the estimated transformation if 3D correspondences are provided (should be > 1). The higher it is, the smaller the covariance will be. With accurate depth estimation, this could be set to 2. For depth estimated by stereo, 4 or more maybe used to ignore large errors of very far points.]
- **Vis/RefineIterations**: "5"  
  [[Vis/EstimationType = 0] Number of iterations used to refine the transformation found by RANSAC. 0 means that the transformation is not refined.]
- **Vis/RoiRatios**: "0.0 0.0 0.0 0.0"  
  [Region of interest ratios [left, right, top, bottom].]
- **Vis/SSC**: "false"  
  [If true, SSC (Suppression via Square Covering) is applied to limit keypoints.]
- **Vis/SubPixEps**: "0.02"  
  [See cv::cornerSubPix().]
- **Vis/SubPixIterations**: "0"  
  [See cv::cornerSubPix(). 0 disables sub pixel refining.]
- **Vis/SubPixWinSize**: "3"  
  [See cv::cornerSubPix().]

## g2o Parameters

- **g2o/Baseline**: "0.075"  
  [When doing bundle adjustment with RGB-D data, we can set a fake baseline (m) to do stereo bundle adjustment (if 0, mono bundle adjustment is done). For stereo data, the baseline in the calibration is used directly.]
- **g2o/Optimizer**: "0"  
  [0=Levenberg 1=GaussNewton]
- **g2o/PixelVariance**: "1.0"  
  [Pixel variance used for bundle adjustment.]
- **g2o/RobustKernelDelta**: "8"  
  [Robust kernel delta used for bundle adjustment (0 means don't use robust kernel). Observations with chi2 over this threshold will be ignored in the second optimization pass.]
- **g2o/Solver**: "0"  
  [0=csparse 1=pcg 2=cholmod 3=Eigen]