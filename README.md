# Nvidia Jetson Xavier NX
[Nvidia Jetson Xavier NX Developer Kit], a guide, may be referred to get familiar with the device.

## Installation
**A | On your system**
- Download the [Jetson Xavier NX Developer Kit SD Card Image].
- Download, install, and launch [Etcher].
- Connect microSD card.
- Launch Etcher, select the image, and flash the card.

**B | On Xavier**
- Mount the microSD card on Xavier.
- Connect hdmi, mouse, keyboard, and power. When connected to power, a green light near the micro-USB will turn on. The monitor should display nvidia logo at the start.
- Setup system configuration; select MODE_15W_6CORE.
- Name the system; keep it simple and short as it would appear on terminal like hawkins@hawkins-desktop.

**C | Making Xavier Ready**
- Press Ctrl+Alt+T to launch the terminal.
- Install X-terminal-emulator, gedit, git.
```
$ sudo apt install terminator
```
- Close the terminal and press Ctrl+Alt+T to relaunch it.
```
$ sudo apt install gedit 
$ sudo apt install git
```
- Use [reef ros autoinstall] on github to install ros-melodic.
```
$ git clone https://github.com/uf-reef-avl/reef_auto_install
$ ./reef_auto_install/autoinstall.sh
```
- Install libraries for realsense camera. 
```
$ sudo apt-get update && sudo apt-get upgrade
$ sudo apt-get install git sudo libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev 
$ sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev cmake 
```
- Fetch [realsense github repository] and build it. More information is provided at [qualcomm robotics].
```
$ git clone https://github.com/IntelRealSense/librealsense.git
$ cd librealsense
$ git reset --hard 61cf21520b4bb29c2c1074f7bad74a6cfcd93ca3
$ ./scripts/setup_udev_rules.sh
$ mkdir build && cd build
$ cmake ../ -DBUILD_EXAMPLES=true -DFORCE_RSUSB_BACKEND=true 
$ make uninstall && make clean 
$ make -j7
$ sudo make install
```
- Install realsense camera drivers.
```
$ sudo apt install ros-melodic-realsense2-*
$ sudo apt install ros-melodic-rgbd-launch
```
- Launch the realsense viewer.
```
$ realsense-viewer
```
- To test, use following commands in another terminal window.
```
$ rs-distance
$ rs-depth
$ rs-save-to-disk
```

## DVO SLAM by UNCC
- To install [uncc visionlab dvo_slam], use following commands in order:
```
$ git clone https://github.com/uncc-visionlab/dvo_slam
$ export ROS_PACKAGE_PATH=`pwd`:$ROS_PACKAGE_PATH
$ rosmake dvo_core dvo_ros dvo_slam dvo_benchmark
```
**Note: The installation instructions on uncc-visionlab github repository are outdated and also the repository link to git clone is different.**


- Git clone following repository anywhere on the system but not in your project.

```
$ git clone https://github.com/DLTcollab/sse2neon
```
- From inside the sse2neon directory, copy sse2neon.h file to dvo_slam/dvo_core/include/dvo/core/

- In sse2neon.h, add following macro:
```
#define _MM_SHUFFLE2(fp1, fp0) \
    (((fp1) << 2) | ((fp0)))
```

- Include sse2neon.h by adding:
```
#include <arm_neon.h>
#include <dvo/core/sse2neon.h>
```
in following files and commenting as follows inline:
```
/dvo_slam/dvo_core/include/dvo/core/surface_pyramid.h
//#include <mmintrin.h>
//#include <emmintrin.h>

/dvo_slam/dvo_core/src/core/math_sse.cpp
//#include <immintrin.h>
//#include <pmmintrin.h>

/dvo_slam/dvo_core/src/core/rgbd_image_sse.cpp
//#include <immintrin.h>
//#include <pmmintrin.h>

/dvo_slam/dvo_core/src/dense_tracking_impl.cpp
//#include <immintrin.h>
//#include <pmmintrin.h>
```

In dvo_slam/dvo_core/CMakeLists.txt, add/update as follows:
```
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(${OpenCV_LIBS})
```

- Following commands were run, not considered stable at the moment.
```
$ sudo apt-get install qemu
$ sudo apt-get install ros-melodic-libg2o
```
- Comment out the astra camera driver since we aim to use realsense camera.
- Delete the dvo_benchmark sub-package to prevent catkin build errors.
- In dvo_ros.launch, add following mapping:
```
<remap from="/camera/color/image_raw" to="/camera/rgb/input_image"/>
<remap from="/camera/depth/image_rect_raw" to="/camera/depth_registered/input_image"/>
<remap from="/camera/aligned_depth_to_color/camera_info" to="/camera/depth_registered/camera_info"/>
<remap from="/camera/color/camera_info" to="/camera/rgb/camera_info"/>
```
Install boost library for C++ as instructed at this link: [install boost]

## Troubleshoot
\# | Problem | Solution | Notes
-- | -- | -- | --
1 | bash: curl: command not found | `$ sudo apt install curl` | --
2 | rosdep not found | `$ sudo apt install python-rosdep` <br> `$ sudo rosdep init` <br> `$ rosdep update`| --
3 | cmake version conflict | Download tar file to [download cmake] and run following commands: <br> `$ tar zxvf cmake-3.20.4.tar.gz` <br> `$ cd cmake-3.20.4` <br> `$ sudo ./bootstrap` <br> `$ sudo make` <br> `$ sudo make install` <br> `$ cmake  --version` | [reinstall cmake]
4 | apt broken | `$ sudo apt update && sudo apt dist-upgrade` | --
5 | rosmake issue on parsing the argument | `$ export ROS_PACKAGE_PATH='pwd':$ROS_PACKAGE_PATH` | [rosmake pocketsphinx issue] <br> [rosmake parsing as stacks issue]
6 | Project 'cv_bridge' specifies '/usr/include/opencv' as an include dir, which is not found. | `$ sudo apt-get install ros-melodic-cv-bridge` <br> `$ cd /usr/include/` <br> `$ sudo ln -s opencv4/ opencv` or `$ sudo ln -s /usr/include/opencv4/opencv2/ /usr/include/opencv` | [cv_bridge opencv issue]
7 | creating symlink failed | `$ sudo ln -sf /usr/include/opencv4/opencv2/ /usr/include/opencv` | [symlink fail]
8 | CMake did not find Sophus | `$ sudo apt-get install ros-melodic-sophus` | --
9 | unrecognized command line option for any of the following: -msse -msse2 -msse3 -msse4 -ftree-vectorize | `$ sudo gedit <dvo_slam package path>/dvo_slam/dvo_core/CMakeLists.txt` <br> comment out set comamnd as follows: <br> `# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3 -msse4 -ftree-vectorize")` | Xavier has an arm architecture and not SSE. By default, the SSE options are enabled. Refer [arm linux g error] and [supercollider build issue with SSE].
10 | fatal error: opencv2/opencv.hpp: No such file or directory #include <opencv2/opencv.hpp> | add following lines: <br> `find_package(OpenCV REQUIRED)` <br><br> `include_directories(<existing directories> ${OpenCV_INCLUDE_DIRS})` <br><br> `target_link_libraries( <existing libraries> ${OpenCV_LIBS})` | [opencv4 opencv conflict] <br> [opencv hpp issue]
11 | fatal error: immintrin.h: No such file or directory #include <immintrin.h> | In specified file, <br> `#include <arm_neon.h>` <br> `// #include <immintrin.h>` | [instrinsic header file issue]
12 | fatal error: mmintrin.h: No such file or directory #include <mmintrin.h> | In specified file, <br> `#include <dvo/core/sse2neon.h>` <br> `// #include <mmintrin.h>` | [sse2neon]
13 | all intrinstic header file errors | `#include <arm_neon.h>` <br> `#include <dvo/core/sse2neon.h>` | [nvidia emmintrin.h not found in TX2]
14 | error: ‘_MM_SHUFFLE2’ was not declared in this scope fac = _mm_mul_pd(s, _mm_shuffle_pd(v1, v1, _MM_SHUFFLE2(0, 0))); | In sse2neon.h, add following macro: <br> `#define _MM_SHUFFLE2(fp1, fp0) `\ <br> `(((fp1) << 2) \| ((fp0)))` | [_MM_SHUFFLE() working]
15 | ERRORG2O missing | `$ sudo apt-get install ros-melodic-libg2o` | --
16 | error: field ‘param_k_’ has incomplete type ‘flann::SearchParams’::flann::SearchParams param_k_; | `$ cd /usr/include/pcl-1.8/pcl/kdtree/` <br> `$ sudo gedit kdtree_flann.h` <br> <br> modify <br> `::flann::SearchParams param_k_;` <br> to <br> `::flann::SearchParams *param_k_;` | [flann::SearchParams issue]
17 | error: field ‘param_radius_’ has incomplete type ‘flann::SearchParams’::flann::SearchParams param_radius_; | `$ cd /usr/include/pcl-1.8/pcl/kdtree/` <br> `$ sudo gedit kdtree_flann.h` <br> <br> modify <br> `::flann::SearchParams param_radius_;` <br> to <br> `::flann::SearchParams *param_radius_;`| [flann::SearchParams issue]
18 | error: ‘CV_BGR2GRAY’ was not declared in this scope cv::cvtColor(rgb, grey, CV_BGR2GRAY); | change `CV_BGR2RGB` to `cv::COLOR_BGR2RGB` | --
19 | warning: libopencv_core.so.3.2 | -- | [opencv bluff] <br> [find opencv]
20 | ERROR [546400366976] (ds5-options.cpp:88) Asic Temperature value is not valid! | -- | [Asic Temperature issue] <br> [Depth module in D435 camera stops working #5209]
21 | point cloud display in rviz | -- | **open** <br> not fixed for current application.
22 | camera_dense_tracking.cpp from dvo_ros execution issue; ROS_WARN not printed | -- | **open**
23 | control_transfer returned error, index: 300, error: Resource temporarily unavailable, number: b | -- | **open** <br> RealSense v2.3.0 currently on Xavier; [control transfer error] discusses this issue for nano. <br> [librealsense2 backend version conflict] gives an insight v4I and RSUSB versions.
24 | Hardware Notification:USB CAM overflow | try relaunching the node, realsense-viewer might be useful | [CAM overflow problem] <br> [D435-USB SCP]
25 | (Confidence, 0) sensor isn't supported by current device! -- Skipping | -- | **open**
26 | CMake error, variables set to NOTFOUND | [cmake, variable set to notfound] | --
27 | build opencv from source | [jetsonhack openCV] | --
28 | No RGB and IMU data on D435i | [D435i USB] | --
29 | Assertion `intensity.size() == depth.size()' failed | -- | **open** <br> [resolution conflict]
30 | rs2_load_json | -- | **open**
31 | Could not find a package configuration file provided by "PCL" | $ sudo apt install ros-<distro>-pcl* | --
32 | dvo_ros/CameraDenseTrackerConfig.h: No such file or directory | chmod +x dvo_slam/dvo_ros/cfg/CameraDenseTracker.cfg | [uncc visionlab dvo_slam issue 42]

## Quick Commands
Commands are for:
- ros version
- opencv version
- find and remove opencv
- cuda version
- USB

```
$ rosversion --distro
$ python3 -c "import cv2; print(cv2.__version__)"
$ sudo find / -name "*opencv*" -exec rm -i {} \;
$ ./nvcc --version
$ ls -l /usr/local | grep cuda
$ lsusb -d 8086: -v | grep -i bcdUSB
$ sudo su
```


## References
1. [Nvidia Jetson Xavier NX Developer Kit]
1. [Jetson Xavier NX Developer Kit SD Card Image]
1. [Etcher]
1. [reef ros autoinstall]
1. [qualcomm robotics]
1. [realsense github repository]
1. [download cmake]
1. [install boost]
1. [reinstall cmake]
1. [uncc visionlab dvo_slam]
1. [rosmake pocketsphinx issue]
1. [rosmake parsing as stacks issue]
1. [cv_bridge opencv issue]
1. [symlink fail]
1. [arm linux g error]
1. [supercollider build issue with SSE]
1. [opencv4 opencv conflict]
1. [opencv hpp issue]
1. [instrinsic header file issue]
1. [sse2neon]
1. [nvidia emmintrin.h not found in TX2]
1. [_MM_SHUFFLE() working]
1. [flann::SearchParams issue]
1. [CV_BGR2RGB issue]
1. [opencv bluff]
1. [find opencv]
1. [Asic Temperature issue]
1. [Depth module in D435 camera stops working #5209]
1. [control transfer error]
1. [librealsense2 backend version conflict]
1. [CAM overflow problem]
1. [D435-USB SCP]
1. [cmake, variable set to notfound]
1. [jetsonhack openCV]
1. [D435i USB]
1. [resolution conflict]
1. [uncc visionlab dvo_slam issue 42]


[//]: # (Hyperlinks to the References)
[Nvidia Jetson Xavier NX Developer Kit]: https://developer.nvidia.com/embedded/jetson-xavier-nx-devkit
[Jetson Xavier NX Developer Kit SD Card Image]: https://developer.nvidia.com/jetson-nx-developer-kit-sd-card-image
[Etcher]: https://www.balena.io/etcher/
[reef ros autoinstall]: https://github.com/uf-reef-avl/reef_auto_install/blob/master/ROS/install_ros_melodic.sh
[qualcomm robotics]: https://developer.qualcomm.com/forum/qdn-forums/hardware/qualcomm-robotics-rb5-development-kit/68234
[realsense github repository]: https://github.com/IntelRealSense/realsense-ros
[download cmake]: https://cmake.org/download/
[install boost]: https://www.osetc.com/en/how-to-install-boost-on-ubuntu-16-04-18-04-linux.html
[reinstall cmake]: https://stackoverflow.com/questions/49859457/how-to-reinstall-the-latest-cmake-version
[uncc visionlab dvo_slam]: https://github.com/uncc-visionlab/dvo_slam
[rosmake pocketsphinx issue]: https://answers.ros.org/question/28315/rosmake-pocketsphinx-issue/
[rosmake parsing as stacks issue]: https://answers.ros.org/question/52666/why-the-following-args-could-not-be-parsed-as-stacks/
[cv_bridge opencv issue]: https://answers.ros.org/question/199279/installation-from-source-fails-because-of-cv_bridge-include-dir/
[symlink fail]: https://stackoverflow.com/questions/37794267/creating-symbolic-link-fails
[arm linux g error]: https://stackoverflow.com/questions/44967149/arm-hisiv300-linux-g-error-unrecognized-command-line-option-msse
[supercollider build issue with SSE]: https://www.mail-archive.com/debian-bugs-dist@lists.debian.org/msg1555254.html
[opencv4 opencv conflict]: https://gitmemory.com/issue/IntelRealSense/realsense-ros/1772/828162504
[opencv hpp issue]: https://stackoverflow.com/questions/63455427/fatal-error-opencv2-opencv-modules-hpp-no-such-file-or-directory-include-ope
[instrinsic header file issue]: https://stackoverflow.com/questions/11228855/header-files-for-x86-simd-intrinsics
[sse2neon]: https://github.com/DLTcollab/sse2neon
[nvidia emmintrin.h not found in TX2]: https://forums.developer.nvidia.com/t/emmintrin-h-not-found-in-tx2/56061
[_MM_SHUFFLE() working]: https://community.intel.com/t5/Intel-C-Compiler/mm-shuffle/m-p/947890
[flann::SearchParams issue]: https://github.com/strands-project/strands_3d_mapping/issues/67
[CV_BGR2RGB issue]: https://github.com/NVIDIA/DALI/issues/735
[opencv bluff]: https://gitmemory.com/issue/IntelRealSense/realsense-ros/1772/828162504
[find opencv]: https://github.com/cggos/DIPDemoQt/issues/1
[Asic Temperature issue]: https://github.com/IntelRealSense/realsense-ros/issues/894
[Depth module in D435 camera stops working #5209]: https://github.com/IntelRealSense/librealsense/issues/5209
[control transfer error]: https://github.com/IntelRealSense/librealsense/issues/6062
[librealsense2 backend version conflict]: https://github.com/IntelRealSense/realsense-ros/issues/1663
[CAM overflow problem]: https://github.com/IntelRealSense/librealsense/issues/3296
[D435-USB SCP]: https://github.com/IntelRealSense/realsense-ros/issues/669
[cmake, variable set to notfound]: https://stackoverflow.com/questions/46584000/cmake-error-variables-are-set-to-notfound
[jetsonhack openCV]: https://github.com/jetsonhacks/buildOpenCVXavier
[D435i USB]: https://github.com/IntelRealSense/realsense-ros/issues/635
[resolution conflict]: https://dev.intelrealsense.com/docs/tuning-depth-cameras-for-best-performance
[uncc visionlab dvo_slam issue 42]: https://github.com/tum-vision/dvo_slam/issues/42
