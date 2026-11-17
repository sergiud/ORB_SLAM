SPDX-FileCopyrightText: 2015-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>

SPDX-License-Identifier: GPL-3.0-or-later

ORB-SLAM Monocular
Current Version: 2.0.0 (see Changelog.md)

ORB-SLAM is a versatile and accurate Monocular SLAM solution able to compute in real-time the camera trajectory and a sparse 3D reconstruction of the scene in a wide variety of environments, ranging from small hand-held sequences to a car driven around several city blocks. It is able to close large loops and perform global relocalisation in real-time and from wide baselines.

See our project webpage: http://webdiis.unizar.es/~raulmur/orbslam/

Related Publications:

[1] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. 

DOI: 10.1109/TRO.2015.2463671.

Link to pdf: http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf


1. License

ORB-SLAM is released under the GNU General Public License version 3 or later,
whose text is in LICENSES/GPL-3.0-or-later.txt. Code derived from other
projects keeps its BSD license in addition, as the SPDX header of each file
states, and the LICENSES directory holds the text of every license.
For a list of all code/library dependencies and their licenses, please see Dependencies.md.

No closed-source license is offered for this version. It combines the
original ORB-SLAM with changes by other authors, so that only all copyright
holders together could grant one. The GPL permits commercial use, provided
that works based on this version are distributed under the GPL as well.

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


2. Prerequisites (dependencies)

2.1 Boost

ORB-SLAM builds with CMake 3.28 or later. The SLAM library runs its threads
with the C++ standard library and needs a compiler that supports C++23. The
command-line program `run_ORB_SLAM` parses its options with
Boost.Program_Options.

	sudo apt-get install libboost-program-options-dev

2.2 OpenCV
We use OpenCV to manipulate images and features. ORB-SLAM needs OpenCV 5.0 or later with the core, calib, imgproc, highgui and features modules. Download and install instructions can be found at: http://opencv.org/

2.3 g2o and Eigen
We use g2o (https://github.com/RainerKuemmerle/g2o) to perform optimizations. Install g2o 2017.7.30 or later with its core, types_sba, types_slam3d and types_sim3 libraries. Build g2o with the same `-march` flags as ORB-SLAM, since g2o chooses the alignment of its Eigen types by them and the two do not link otherwise.
ORB-SLAM needs Eigen 3.4.1 or later, up to version 5. g2o needs BLAS and LAPACK in addition.

	sudo apt-get install libblas-dev
	sudo apt-get install liblapack-dev
	sudo apt-get install libeigen3-dev

2.4 DBoW2 (included in Thirdparty)
We make use of some components of the DBoW2 library (see original at https://github.com/dorian3d/DBoW2) for place recognition and feature matching. ORB-SLAM builds it along with the library.

2.5 ng-log
ORB-SLAM logs its messages and checks its preconditions with the experimental C++20 interface of ng-log (https://github.com/ng-log/ng-log). Install ng-log 0.9.0 or later built with that interface, which CMake finds as the `experimental` component of the ng-log package. `run_ORB_SLAM` writes the messages to the standard error.


3. Installation

  1) Install the dependencies of section 2, g2o as section 2.3 describes.

  2) Clone the repository:

		git clone https://github.com/raulmur/ORB_SLAM.git ORB_SLAM

  3) Build ORB-SLAM and run its tests. In the ORB_SLAM root execute:

		cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-march=native
		cmake --build build
		ctest --test-dir build

  4) Optionally install the library, its headers, `run_ORB_SLAM` and the CMake package `orb_slam`, which `find_package (orb_slam)` finds. The variables of CMake's `GNUInstallDirs` module, such as `CMAKE_INSTALL_LIBDIR`, choose the directories below the prefix:

		cmake --install build --prefix PATH_TO_INSTALLATION

4. Usage

  1) Uncompress the vocabulary in `Data/ORBvoc.txt.tar.gz`, since ORB-SLAM reads the text file much faster.

  2) Run ORB-SLAM on a video with the calibration of its camera:

		build/run_ORB_SLAM --vocabulary Data/ORBvoc.txt --calibration CALIBRATION_FILE --input VIDEO

  The input is anything that OpenCV's `cv::VideoCapture` opens, such as a video file or a sequence of images named by a pattern like `rgb/%d.png`. A window shows the tracked features unless `--no-video` is given. With `--trajectory FILE`, ORB-SLAM writes the pose of each keyframe at the end, one line each with the time stamp, the position and the orientation as a quaternion in the order x, y, z, w. `run_ORB_SLAM --help` lists all options.

  3) The calibration file is a YAML file of OpenCV's `cv::FileStorage` with the camera matrix and the distortion coefficients of OpenCV's camera model, 4 or 5 values ordered k1, k2, p1, p2 and k3:

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

5. Failure Modes

You should expect to achieve good results in sequences similar to those in which we show results in our paper [1], in terms of camera movement and texture in the environment. In general our Monocular SLAM solution is expected to have a bad time in the following situations:
- Pure rotations in exploration
- Low texture environments
- Many (or big) moving objects, especially if they move slowly.

The system is able to initialize from planar and non-planar scenes. In the case of planar scenes, depending on the camera movement relative to the plane, it is possible that the system refuses to initialize, see the paper [1] for details. 

6. Need Help?

If you have any trouble installing or running ORB-SLAM, contact the authors.

