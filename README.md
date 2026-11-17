<!--
SPDX-FileCopyrightText: 2015-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
SPDX-FileCopyrightText: 2026 Sergiu Deitsch <sergiu.deitsch@gmail.com>

SPDX-License-Identifier: GPL-3.0-or-later
-->

# ORB-SLAM Monocular

ORB-SLAM computes the trajectory of a single camera and a sparse 3D map of the
scene in real time. It closes large loops and relocalizes from wide baselines,
in scenes from hand-held sequences to city blocks driven by car. Raúl
Mur-Artal, J. M. M. Montiel and Juan D. Tardós created it [1], with place
recognition by DBoW2 of Dorian Gálvez-López [2].

## Motivation

This is a fork of the [original ORB-SLAM](https://github.com/raulmur/ORB_SLAM)
that is free of ROS. It builds as a plain CMake project with C++23, OpenCV 5
and current releases of g2o and Eigen, which the original no longer builds
with, and logs through ng-log. It fixes many defects of
the original, such as data races between its threads, memory leaks of culled
keyframes and map points, and errors in initialization, relocalization and
loop closing.

## Requirements

* CMake 3.28 and a C++23 compiler
* OpenCV 5.0 with the core, calib, imgproc, highgui and features modules
* [g2o](https://github.com/RainerKuemmerle/g2o) 2017.7.30 with the core,
  types_sba, types_slam3d and types_sim3 libraries, built with the same
  `-march` flags as ORB-SLAM, since the alignment of its Eigen types depends
  on them
* Eigen 3.4.1 up to version 5
* [ng-log](https://github.com/ng-log/ng-log) 0.9.0 with its experimental C++20
  interface
* Boost.Program_Options 1.72

DBoW2 is included in `Thirdparty/DBoW2` and built along with the library.

## Getting Started

Build ORB-SLAM, run its tests and unpack the vocabulary:

```sh
git clone https://github.com/sergiud/ORB_SLAM.git
cd ORB_SLAM
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-march=native
cmake --build build
ctest --test-dir build
tar -xzf Data/ORBvoc.txt.tar.gz -C Data
```

Then run it on a video:

```sh
build/run_ORB_SLAM --vocabulary Data/ORBvoc.txt --calibration camera.yml video.mp4
```

The input is anything that OpenCV's `cv::VideoCapture` opens, such as a video
file or images named by a pattern like `rgb/%d.png`. The calibration is a
`cv::FileStorage` file with the camera matrix and 4 or 5 distortion
coefficients ordered k1, k2, p1, p2 and k3:

```yaml
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
```

Tracking works best with texture, translation at the start and few moving
objects. Pure rotations during exploration lose it, and planar scenes may
not initialize for some camera motions [1].

## Advanced Usage

`--trajectory <file>` writes the pose of each keyframe at the end, one line
each with the time stamp, the position and the orientation as a quaternion
x, y, z, w. `--no-video` hides the window of tracked features, and `--help`
lists the options of the feature extraction and the motion model.

`cmake --install build --prefix <prefix>` installs the library, its headers,
`run_ORB_SLAM` and a CMake package into the directories of `GNUInstallDirs`.
Other projects then link against it:

```cmake
find_package (orb_slam 2.0 REQUIRED)
target_link_libraries (app PRIVATE ORB_SLAM::ORB_SLAM)
```

`Data/Settings.yaml` shows the settings the tracker reads when the library
constructs it from a settings file.

## Licensing

ORB-SLAM is released under the
[GNU General Public License version 3 or later](LICENSES/GPL-3.0-or-later.txt).
Code derived from other projects keeps its BSD license in addition, as the
SPDX header of each file states. [Dependencies.md](Dependencies.md) lists the
licenses of the dependencies.

No closed-source license is offered for this version. It combines the original
ORB-SLAM with changes by other authors, so that only all copyright holders
together could grant one. The GPL permits commercial use, provided that works
based on this version are distributed under the GPL as well.

If you use ORB-SLAM in academic work, please cite [1], and [2] for place
recognition:

1. Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. ORB-SLAM: A Versatile
   and Accurate Monocular SLAM System. *IEEE Transactions on Robotics*, 31(5),
   1147–1163, 2015. [doi:10.1109/TRO.2015.2463671](https://doi.org/10.1109/TRO.2015.2463671)
2. Dorian Gálvez-López and Juan D. Tardós. Bags of Binary Words for Fast Place
   Recognition in Image Sequences. *IEEE Transactions on Robotics*, 28(5),
   1188–1197, 2012. [doi:10.1109/TRO.2012.2197158](https://doi.org/10.1109/TRO.2012.2197158)
