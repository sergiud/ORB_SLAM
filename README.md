<!--
SPDX-FileCopyrightText: 2015-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>

SPDX-License-Identifier: GPL-3.0-or-later
-->

##Check out our new [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) (Monocular, Stereo and RGB-D)
---
# ORB-SLAM Monocular
**Authors:** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2))

**Current version:** 2.0.0 (see [Changelog.md](https://github.com/raulmur/ORB_SLAM/blob/master/Changelog.md))

ORB-SLAM is a versatile and accurate Monocular SLAM solution able to compute in real-time the camera trajectory and a sparse 3D reconstruction of the scene in a wide variety of environments, ranging from small hand-held sequences to a car driven around several city blocks. It is able to close large loops and perform global relocalisation in real-time and from wide baselines.

See our project webpage: http://webdiis.unizar.es/~raulmur/orbslam/

###Related Publications:

[1] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. (2015 IEEE Transactions on Robotics **Best Paper Award**). **[PDF](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf)**.

[2] Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp.  1188-1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**.


#1. License

ORB-SLAM is released under a [GPLv3 license](https://github.com/raulmur/ORB_SLAM/blob/master/License-gpl.txt). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/raulmur/ORB_SLAM/blob/master/Dependencies.md).

For a closed-source version of ORB-SLAM for commercial purposes, please contact the authors: orbslam (at) unizar (dot) es. 

If you use ORB-SLAM in an academic work, please cite:

    @article{murAcceptedTRO2015,
      title={{ORB-SLAM}: a Versatile and Accurate Monocular {SLAM} System},
      author={Mur-Artal, Ra\'ul, Montiel, J. M. M. and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={31},
      number={5},
      pages={1147--1163},
      doi = {10.1109/TRO.2015.2463671},
      year={2015}
     }


#2. Prerequisites (dependencies)

##2.1 Boost

The SLAM library runs its threads with the C++ standard library and needs a
compiler that supports C++23. The command-line program `run_ORB_SLAM` parses
its options with Boost.Program_Options.

	sudo apt-get install libboost-program-options-dev

##2.2 OpenCV
We use OpenCV to manipulate images and features. ORB-SLAM needs OpenCV 5.0 or later with the core, calib, imgproc, highgui and features modules. Download and install instructions can be found at: http://opencv.org/

##2.3 g2o and Eigen
We use g2o (https://github.com/RainerKuemmerle/g2o) to perform optimizations. Install g2o 2017.7.30 or later with its core, types_sba, types_slam3d and types_sim3 libraries. Build g2o with the same `-march` flags as ORB-SLAM, since g2o chooses the alignment of its Eigen types by them and the two do not link otherwise.
ORB-SLAM needs Eigen 3.4.1 or later, up to version 5. g2o needs BLAS and LAPACK in addition.

	sudo apt-get install libblas-dev
	sudo apt-get install liblapack-dev
	sudo apt-get install libeigen3-dev

##2.4 DBoW2 (included in Thirdparty)
We make use of some components of the DBoW2 library (see original at https://github.com/dorian3d/DBoW2) for place recognition and feature matching. ORB-SLAM builds it along with the library.

##2.6 ng-log
ORB-SLAM logs its messages and checks its preconditions with the experimental C++20 interface of ng-log (https://github.com/ng-log/ng-log). Install ng-log 0.9.0 or later built with that interface, which CMake finds as the `experimental` component of the ng-log package. `run_ORB_SLAM` writes the messages to the standard error.


#3. Installation

1. Install the dependencies of section 2, g2o as section 2.3 describes.

2. Clone the repository:

		git clone https://github.com/raulmur/ORB_SLAM.git ORB_SLAM

3. Build ORB-SLAM and run its tests. In the ORB_SLAM root execute:

		cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-march=native
		cmake --build build
		ctest --test-dir build

4. Optionally install the library, its headers and the CMake package `orb_slam`, which `find_package (orb_slam)` finds:

		cmake --install build --prefix PATH_TO_INSTALLATION

#4. Usage

1. Uncompress the vocabulary in `Data/ORBvoc.txt.tar.gz`, since ORB-SLAM reads the text file much faster.

2. Run ORB-SLAM on a video with the calibration of its camera:

		build/run_ORB_SLAM --vocabulary Data/ORBvoc.txt --calibration CALIBRATION_FILE --input VIDEO

  The input is anything that OpenCV's `cv::VideoCapture` opens, such as a video file or a sequence of images named by a pattern like `rgb/%d.png`. A window shows the tracked features unless `--no-video` is given. With `--trajectory FILE`, ORB-SLAM writes the pose of each keyframe at the end, one line each with the time stamp, the position and the orientation as a quaternion in the order x, y, z, w. `run_ORB_SLAM --help` lists all options.

3. The calibration file is a YAML file of OpenCV's `cv::FileStorage` with the camera matrix and the distortion coefficients of OpenCV's camera model, 4 or 5 values ordered k1, k2, p1, p2 and k3:

		%YAML:1.0
		camera_matrix: !!opencv-matrix
		   rows: 3
		   cols: 3
		   dt: d
		   data: [ 481.2, 0., 319.5, 0., 480., 239.5, 0., 0., 1. ]
		distortion_coefficients: !!opencv-matrix
		   rows: 1
		   cols: 4
		   dt: d
		   data: [ 0., 0., 0., 0. ]

  `Data/Settings.yaml` shows the settings that the tracker reads when the library constructs it from a settings file.

#5. Failure Modes

You should expect to achieve good results in sequences similar to those in which we show results in our paper [1], in terms of camera movement and texture in the environment. In general our Monocular SLAM solution is expected to have a bad time in the following situations:
- No translation at system initialization (or too much rotation).
- Pure rotations in exploration.
- Low texture environments.
- Many (or big) moving objects, especially if they move slowly.

The system is able to initialize from planar and non-planar scenes. In the case of planar scenes, depending on the camera movement relative to the plane, it is possible that the system refuses to initialize, see the paper [1] for details. 

