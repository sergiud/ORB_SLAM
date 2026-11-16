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

ORB-SLAM is released under a GPLv3 license.  
For a list of all code/library dependencies (and associated licenses), please see Dependencies.md.

For a closed-source version of ORB-SLAM for commercial purposes, please contact the authors. 

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

The SLAM library runs its threads with the C++ standard library and needs a
compiler that supports C++23. The command-line program run_ORB_SLAM parses
its options with Boost.Program_Options, and the ROS node starts its threads
with Boost.Thread.

	sudo apt-get install libboost-program-options-dev libboost-thread-dev

2.2 ROS
We use ROS to receive images from the camera or from a recorded sequence (rosbag), and for visualization (rviz, image_view). 
We have tested ORB-SLAM in Ubuntu 12.04 with ROS Fuerte, Groovy and Hydro; and in Ubuntu 14.04 with ROS Indigo. 
If you do not have already installed ROS in your computer, we recommend you to install the Full-Desktop version of ROS Fuerte (http://wiki.ros.org/fuerte/Installation/Ubuntu).

If you use ROS Indigo, remove the depency of opencv2 in the manifest.xml.

2.3 OpenCV
We use OpenCV to manipulate images and features. ORB-SLAM needs OpenCV 5.0 or later with the core, calib, imgproc, highgui and features modules.
Download and install instructions can be found at: http://opencv.org/

2.4 g2o and Eigen
We use g2o (https://github.com/RainerKuemmerle/g2o) to perform optimizations. Install g2o 2017.7.30 or later with its core, types_sba, types_slam3d and types_sim3 libraries. Build g2o with the same -march flags as ORB-SLAM, since g2o chooses the alignment of its Eigen types by them and the two do not link otherwise.
ORB-SLAM needs Eigen 3.4.1 or later, up to version 5. g2o needs BLAS and LAPACK in addition.

	sudo apt-get install libblas-dev
	sudo apt-get install liblapack-dev
	sudo apt-get install libeigen3-dev

2.5 DBoW2 (included in Thirdparty)
We make use of some components of the DBoW2 library (see original at https://github.com/dorian3d/DBoW2) for place recognition and feature matching. There are no additional dependencies to compile DBoW2.


3. Installation

  1) Make sure you have installed ROS and all library dependencies (boost, eigen3, opencv, blas, lapack).

  2) Clone the repository:

		git clone https://github.com/raulmur/ORB_SLAM.git ORB_SLAM
		
  3) Add the path where you cloned ORB-SLAM to the `ROS_PACKAGE_PATH` environment variable. To do this, modify your .bashrc and add at the bottom the following line (replace  PATH_TO_PARENT_OF_ORB_SLAM):

		export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH_TO_PARENT_OF_ORB_SLAM

  4) Install g2o as section 2.4 describes.

  5) Build DBoW2. Go into Thirdparty/DBoW2/ and execute:

		mkdir build
		cd build
		cmake .. -DCMAKE_BUILD_TYPE=Release
		make  

	Tip: Set your favorite compilation flags in line 4 and 5 of `Thirdparty/DBoW2/CMakeLists.txt` (by default -03 -march=native)

  6) Build ORB_SLAM. In the ORB_SLAM root execute:

	If you use ROS Indigo, remove the depency of opencv2 in the manifest.xml.

		mkdir build
		cd build
		cmake .. -DROS_BUILD_TYPE=Release
		make

	Tip: Set your favorite compilation flags in line 12 and 13 of* `./CMakeLists.txt` (by default -03 -march=native)

4. Usage

See section 5 to run the Example Sequence.

  1) Launch ORB-SLAM from the terminal (`roscore` should have been already executed):

		rosrun ORB_SLAM ORB_SLAM PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE

  You have to provide the path to the ORB vocabulary and to the settings file. The paths must be absolute or relative   to the ORB_SLAM directory.  
  We already provide the vocabulary file we use in `ORB_SLAM/Data/ORBvoc.txt.tar.gz`. Uncompress the file, as it will be loaded much faster.

  2) The last processed frame is published to the topic `/ORB_SLAM/Frame`. You can visualize it using `image_view`:

		rosrun image_view image_view image:=/ORB_SLAM/Frame _autosize:=true

  3) The map is published to the topic `/ORB_SLAM/Map`, the current camera pose and global world coordinate origin are sent through `/tf` in frames `/ORB_SLAM/Camera` and `/ORB_SLAM/World` respectively.  Run `rviz` to visualize the map:
	
	in ROS Fuerte:

		rosrun rviz rviz -d Data/rviz.vcg

	in ROS Groovy or a newer version:

		rosrun rviz rviz -d Data/rviz.rviz

  4) ORB_SLAM will receive the images from the topic `/camera/image_raw`. You can now play your rosbag or start your camera node. 
If you have a sequence with individual image files, you will need to generate a bag from them. We provide a tool to do that: https://github.com/raulmur/BagFromImages.


Tip: Use a roslaunch to launch `ORB_SLAM`, `image_view` and `rviz` from just one instruction. We provide an example:

in ROS Fuerte:

	roslaunch ExampleFuerte.launch

in ROS Groovy or a newer version:

	roslaunch ExampleGroovyOrNewer.launch


5. Example Sequence
We provide the settings and the rosbag of an example sequence in our lab. In this sequence you will see a loop closure and two relocalisation from a big viewpoint change.

  1) Download the rosbag file:  
	http://webdiis.unizar.es/~raulmur/orbslam/downloads/Example.bag.tar.gz. 

	Alternative link: https://drive.google.com/file/d/0B8Qa2__-sGYgRmozQ21oRHhUZWM/view?usp=sharing

	Uncompress the file.
  
  2) Launch ORB_SLAM with the settings for the example sequence. You should have already uncompressed the vocabulary file (`/Data/ORBvoc.txt.tar.gz`)

  in ROS Fuerte:

	  roslaunch ExampleFuerte.launch

  in ROS Groovy or newer versions:

	  roslaunch ExampleGroovyOrNewer.launch

  3) Once the ORB vocabulary has been loaded, play the rosbag (press space to start):

	rosbag play --pause Example.bag


6. The Settings File

ORB_SLAM reads the camera calibration and setting parameters from a YAML file. We provide an example in `Data/Settings.yaml`, where you will find all parameters and their description. We use the camera calibration model of OpenCV.

Please make sure you write and call your own settings file for your camera (copy the example file and modify the calibration)

7. Failure Modes

You should expect to achieve good results in sequences similar to those in which we show results in our paper [1], in terms of camera movement and texture in the environment. In general our Monocular SLAM solution is expected to have a bad time in the following situations:
- Pure rotations in exploration
- Low texture environments
- Many (or big) moving objects, especially if they move slowly.

The system is able to initialize from planar and non-planar scenes. In the case of planar scenes, depending on the camera movement relative to the plane, it is possible that the system refuses to initialize, see the paper [1] for details. 

8. Need Help?

If you have any trouble installing or running ORB-SLAM, contact the authors.

