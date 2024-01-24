# Workspace Setup

## Build instructions

> last update: 20240123

### Requirements

This part shows the requirements you could/should configure first before you build the `feeding task` workspace.

1. Ubuntu: 20.04 LTS
2. Nvidia driver: 535 or other suitable version
3. cuda: 11.1/11.0
4. anaconda: any version
5. ROS: noetic



> last update: 20231219

### Package list (catkin list)

All of those packages can be download at [zingff](https://github.com/zingff).  You can take a glimpse at the following list to know what each package is for (see in this [part](###Recommended build procedure ). Note that most packages have their official website and updated release, but some of their source code is modified due to my custom usage (marked in **blod**), so it is recommended to download all those packages from my [GitHub account](https://github.com/zingff) unless you know exactly what you are doing. Any package with a different version (especially `tesseract`-related packages) would potentially lead to unknown error and build failure. I will list the official GitHub for those third-party packages. Custom packages are in *Italic*.

1. *anygrasp_generation*
2. apriltag
3. **apriltag_ros**
4. *face_detection*
5. *for_test*
6. gazebo_version_helpers
7. gazebo_grasp_plugin
8. **geometry2**
9. *gpd_pick_and_place*
10. ifopt
11. **kinova_vision**
12. kortex_control
13. **kortex_description**
14. gen3_lite_gen3_lite_2f_move_it_config
15. **gen3_move_it_config**
16. gen3_robotiq_2f_140_move_it_config
17. **gen3_robotiq_2f_85_move_it_config**
18. **kortex_driver**
19. *admittance_controller_d*
20. *door_open_task*
21. kortex_examples
22. kortex_gazebo
23. **kortex_move_it_config**
24. pinocchio
25. *pr_assets*
26. roboticsgroup_upatras_gazebo_plugins
27. ros_industrial_cmake_boilerplate
28. descartes_light
29. osqp
30. osqp_eigen
31. ruckig
32. **tesseract_common**
33. tesseract_command_language
34. tesseract_msgs
35. **tesseract_support**
36. tesseract_scene_graph
37. tesseract_collision
38. tesseract_srdf
39. tesseract_time_parameterization
40. tesseract_urdf
41. tesseract_state_solver
42. **tesseract_kinematics**
43. tesseract_environment
44. tesseract_visualization
45. tf2_msgs
46. tf2
47. tf2_bullet
48. tf2_eigen
49. tf2_py
50. tf2_ros
51. tf2_geometry_msgs
52. tf2_kdl
53. test_tf2
54. tf2_sensor_msgs
55. tf2_tools
56. trajopt_utils
57. trajopt_sco
58. **trajopt**
59. trajopt_ifopt
60. trajopt_sqp
61. **tesseract_motion_planners**
62. **tesseract_task_composer**
63. **tesseract_examples**
64. tesseract_qt
65. tesseract_rosutils
66. tesseract_monitoring
67. *kortex_motion_planning*
68. *feeding_task*
69. tesseract_planning_server
70. **tesseract_ros_examples**
71. tesseract_rviz
72. vhacd
73. *yolo_sam*

### How to download those repos from github via SSH

1. Configure your github account locally, see [tutorial](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent).

   ```bash
   git config --global user.name wdsb
   git config --global user.email wdsb@arl.com
   ssh-keygen -t ed25519 -C wdsb@arl.com
   eval "$(ssh-agent -s)"
   ssh-add ~/.ssh/id_ed25519
   cat ~/.ssh/id_ed25519.pub  # then add a ssh key
   ssh -T git@github.com  # for test
   ```

2. Run `download_repos.sh`. or use a `wstool` method (recommended):

   ```bash
   cd ~/mealAssistiveRobot/sla_ws/src
   git clone https://github.com/zingff/sla_feeding_rosinstalls
   wstool init # exclude if already have .rosinstall
   wstool merge ~/mealAssistiveRobot/sla_ws/src/sla_rosinstalls/sla_feeding.ssh.rosinstall
   wstool up
   ```

   After all repositories cloned successfully, you can begin to build the workspace for feeding task.

### Build setup

1. **OPTIONAL**: additional cmake args: (skip)
   `-DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.8 -DPYTHON_LIBRARY_DIR=/usr/lib/x86_64-linux-gnu`

2. **BUILDING TOOL**: install `python3-catkin-tools`, or you can use `catkin_make`.

3. **DEPENDENCIES**: make sure all deps are satisfied before building.

Other general steps for building a ROS catkin workspace (such as `rosdep`, `wstool` etc,.) are to be omitted in this tutorial.

### Recommended build procedure

> last update: 20240124

> last update: 20231220

You can build the workspace following the following order. Or you can build the whole workspace one-off, but I never try.

1. `apriltag`: pose estimation for QR code

   apriltag, apriltag_ros

   Note: then skiplist apriltag

2. `pinocchio`: dynamic modeling and computation

   pinocchio

   Note: then skiplist pinocchio

   Note: 1 and 2 are exchangeable.

   Todo: take care of the source code, modify gitignore

3. `ros_kortex`: ros package for interacting with Kinova Gen3, just pay attention to `kortex_driver`. Note that `kortex_driver` is modified for some custom reason, do not use the [official version](https://github.com/Kinovarobotics/ros_kortex) since it may not be compatible with the feeding-related codes.

   see [how to build](https://github.com/Kinovarobotics/ros_kortex#build):

   ```bash
   sudo apt install python3 python3-pip
   sudo python3 -m pip install conan==1.59
   conan config set general.revisions_enabled=1
   conan profile new default --detect > /dev/null
   conan profile update settings.compiler.libcxx=libstdc++11 default
   mkdir -p catkin_workspace/src
   cd mealAssistiveRobot/sla_ws
   rosdep install --from-paths src --ignore-src -y -r  # recommend to run this line at very beginning
   ```

   gazebo_version_helpers gazebo_grasp_plugin kortex_control kortex_description gen3_lite_gen3_lite_2f_move_it_config gen3_move_it_config gen3_robotiq_2f_140_move_it_config gen3_robotiq_2f_85_move_it_config kortex_driver kortex_examples kortex_gazebo kortex_move_it_config roboticsgroup_upatras_gazebo_plugins

4. `geometry2`: to shield some incessant TF warnings

   geometry2 tf2_msgs tf2 tf2_bullet tf2_eigen tf2_py tf2_ros tf2_geometry_msgs tf2_kdl test_tf2 tf2_sensor_msgs tf2_tools

5. `tesseract`: libraries for motion planning

   If any error occurs, see [official tutorial](https://tesseract-docs.readthedocs.io/en/latest/_source/intro/getting_started_doc.html).

   ifopt ros_industrial_cmake_boilerplate descartes_light osqp osqp_eigen ruckig tesseract_common tesseract_command_language tesseract_msgs tesseract_support tesseract_scene_graph tesseract_collision tesseract_srdf tesseract_time_parameterization tesseract_urdf tesseract_state_solver tesseract_kinematics tesseract_environment tesseract_visualization trajopt_utils trajopt_sco trajopt trajopt_ifopt trajopt_sqp tesseract_motion_planners tesseract_task_composer tesseract_examples tesseract_qt tesseract_rosutils tesseract_monitoring tesseract_planning_server tesseract_ros_examples tesseract_rviz vhacd 

6. `ros_kortex_vision`: for utilizing depth and color streams from Kinova Gen3

   kinova_vision

7. `face_detection`: for face detection and mouth pose estimation based on `dlib`

   face_detection

   ```bash
   mkdir build
   cd build/
   cmake ..
   cmake --build . --config Release
   make -j16
   make install
   ```

   Note: install [dlib](http://dlib.net/compile.html) first, recommend to install with source code. 19.24 is tested.

   Note: 6 and 7 are exchangeable.

8. `pr_assets`: include some useful assets.

   pr_assets

   Note: this package is optional

9. `door_open`: open door with admittance controller

   door_open_task admittance_controller_d

10. `grasp generation`: generate grasps for any objects

    anygrasp_generation

11. `object_grasping`: for object grasping, independent of feeding task. ignore. 

    gpd_pick_and_place

12. `motion planning`: formulate optimization problem based on `tesseract`; execute motion after solving the problem; real-time collision detection; other functional modules used in feeding task.

    kortex_motion_planning 

    Note: install `qcustomplot` first, ignore, fixed

    Note: build `srv` first

    Todo: modify to one-off build.

13. `task-level package`: most focuses on task manager for feeding task, a finite state machine is empolyed; voice module is to be implemented within this package.

    feeding_task

14. ~~pinocchio (ahead, skip)~~

15. ~~rest (skip)~~
    ~~darknet_ros (not used yet)~~
    ~~segment_anything (deprecated)~~
    ~~Grounded-Segment-Anything (python only)~~
    ~~sla_feeding (deprecated)~~

    ~~kortex_utilities (deprecated)~~

    ~~object_detection (deprecated)~~

After build the workspace, some tips for convenience:

1. run

```
ln -s mealAssistiveRobot/sla_ws/src/feeding_task/bash ~/bash
```

2. add source file in `.bashrc`.
3. create several conda virtual environments, see [AnyGrasp](###AnyGrasp).
   1. anygrasp: if you want to use AnyGrasp for grasp generation
   2. sam: if you want to use Segment Anything for object segmentation.


### Slave diary

20231220: prof. zhang learned a lot in Singapore, and shows off every corner of the lab. funny huckster... prof. chen proposes the framework for my paper hhhhh, jile? fanzheng wo buji.

20231225：什么都不会还能当教授，这辈子算是开了眼了，此生无憾。

20231226：陈老师今天上午又来实验室作遗言宣告，像领导视察工作，每个人都要说两句，是怕活不到明天了吗？

20240123：真是无语，之前cwd一直拖着我不让我去医院做实验，昨天开足会索尼一说要给24年合作的60w这沙比就松口了，让我年前去医院做实验，我看他是想钱想疯了。

20240124: meet ugly guy when peeing.....真晦气

## Previous build instructions

Those instruction are out of date, deprecated. for reference only.

### Build Steps (last update 20231001)

> last build: 20230512
>
> last build: 20230605
>
> last build: 20231001

```bash
$ catkin list 
- apriltag
- apriltag_ros
- gazebo_version_helpers
- gazebo_grasp_plugin
- ifopt
- kinova_vision
- kortex_control
- kortex_description
- gen3_lite_gen3_lite_2f_move_it_config
- gen3_move_it_config
- gen3_robotiq_2f_140_move_it_config
- gen3_robotiq_2f_85_move_it_config
- kortex_driver
- kortex_examples
- kortex_gazebo
- kortex_move_it_config
- kortex_param
- roboticsgroup_upatras_gazebo_plugins
- ros_industrial_cmake_boilerplate
- descartes_light
- osqp
- osqp_eigen
- ruckig
- sla_feeding
- tesseract_common
- tesseract_command_language
- tesseract_msgs
- tesseract_support
- tesseract_geometry
- tesseract_scene_graph
- tesseract_collision
- tesseract_srdf
- tesseract_time_parameterization
- tesseract_urdf
- tesseract_state_solver
- tesseract_kinematics
- tesseract_environment
- tesseract_visualization
- trajopt_utils
- trajopt_sco
- trajopt
- trajopt_ifopt
- trajopt_sqp
- tesseract_motion_planners
- tesseract_task_composer
- tesseract_examples
- tesseract_qt
- tesseract_rosutils
- tesseract_monitoring
- tesseract_planning_server
- tesseract_ros_examples
- tesseract_rviz
- vhacd

-DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.8 -DPYTHON_LIBRARY_DIR=/usr/lib/x86_64-linux-gnu

1. conda deactivate
apriltag apriltag_ros

2. conda deactivate chmod +x home_robot.py
gazebo_version_helpers gazebo_grasp_plugin kortex_control kortex_description gen3_lite_gen3_lite_2f_move_it_config gen3_move_it_config gen3_robotiq_2f_140_move_it_config gen3_robotiq_2f_85_move_it_config kortex_driver kortex_examples kortex_gazebo kortex_move_it_config roboticsgroup_upatras_gazebo_plugins

3. conda deactivate 
geometry2 tf2_msgs tf2 tf2_bullet tf2_eigen tf2_py tf2_ros tf2_geometry_msgs tf2_kdl test_tf2 tf2_sensor_msgs tf2_tools

4. conda deactivate 
ifopt ros_industrial_cmake_boilerplate descartes_light osqp osqp_eigen ruckig tesseract_common tesseract_command_language tesseract_msgs tesseract_support tesseract_scene_graph tesseract_collision tesseract_srdf tesseract_time_parameterization tesseract_urdf tesseract_state_solver tesseract_kinematics tesseract_environment tesseract_visualization trajopt_utils trajopt_sco trajopt trajopt_ifopt trajopt_sqp tesseract_motion_planners tesseract_task_composer tesseract_examples tesseract_qt tesseract_rosutils tesseract_monitoring tesseract_planning_server tesseract_ros_examples tesseract_rviz vhacd 

5. conda deactivate
kinova_vision

6. conda deactivate
face_detection

7. conda deactivate
pr_assets

8. conda deactivate
door_open: catkin build door_open_task admittance_controller_d plot (to check)

9. conda deactivate
anygrasp_generation feeding_task object_grasping (gpd_pick_and_place)
kortex_motion_planning

10. conda deactivate
kortex_utilities
object_detection

11. conda deactivate
pinocchio

11. rest
darknet_ros
segment_anything
sla_feeding

note: 
catkin config --skiplist apriltag pinocchio ifopt ros_industrial_cmake_boilerplate descartes_light osqp osqp_eigen ruckig tesseract_common tesseract_command_language tesseract_msgs tesseract_support tesseract_scene_graph tesseract_collision tesseract_srdf tesseract_time_parameterization tesseract_urdf tesseract_state_solver tesseract_kinematics tesseract_environment tesseract_visualization trajopt_utils trajopt_sco trajopt trajopt_ifopt trajopt_sqp tesseract_motion_planners tesseract_task_composer tesseract_examples tesseract_qt tesseract_rosutils tesseract_monitoring tesseract_planning_server tesseract_ros_examples tesseract_rviz vhacd geometry2 tf2_msgs tf2 tf2_bullet tf2_eigen tf2_py tf2_ros tf2_geometry_msgs tf2_kdl test_tf2 tf2_sensor_msgs tf2_tools
```

### Build Steps (updated 20230605)

- conda deactivate
- build apriltag
- build apriltag_ros
- build ros_kortex and tesseract
- build kortex_motion_planning
- build sla_feeding

### Build steps (update 20230512)

- (Currently necessary) Deactivate conda venv and use the default env:

```bash
conda deactivate
```

- (Currently necessary) Add addtional cmake args

```bash
catkin config --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3.8 -DPYTHON_INCLUDE_DIR=/usr/include/python3.8
```

- Run the following command to automatically install all debian dependencies listed in each `package.xml` file:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

-r: continue installing despite errors

```bash
rosdep install -y --from-paths ~/tesseract_ws/src --ignore-src --rosdistro noetic
```

- Build `apriltag` and `apriltag_ros` separately:

```bash
catkin build apriltag
catkin build apriltag_ros
```

and then add them to the catkin build `skiplist`:

```bash
catkin config --skiplist apriltag apriltag_ros
```

- Build other package:

```bash
catkin build
```

## Troubleshooting

### Tesseract

1. `tiff` error or something similar to the following

```bash
/usr/bin/ld: /home/zing/anaconda3/lib/libQt5Core.so.5.15.2: undefined reference to `std::__exception_ptr::exception_ptr::_M_addref()@CXXABI_1.3.13'
/usr/bin/ld: /lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p: undefined reference to `TIFFSetErrorHandler@LIBTIFF_4.0'
/usr/bin/ld: /lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p: undefined reference to `TIFFSetDirectory@LIBTIFF_4.0'
/usr/bin/ld: /lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p: undefined reference to `TIFFReadScanline@LIBTIFF_4.0'
/usr/bin/ld: /lib/x86_64-linux-gnu/libvtkIOImage-7.1.so.7.1p: undefined reference to `TIFFNumberOfTiles@LIBTIFF_4.0'
```

- deactivate conda and rebuild
- If it is caused by ubuntu system upgrade and therefore conflicts appear between `anaconda` and the system, try delete those lines related to anaconda in `.bashrc` and build `tesseract_qt` isolatedly bu running `catkin build tesseract_qt`. Note that you need to `catkin clean tesseract_qt` and then rebuild.

```bash
__conda_setup="$('/home/zing/anaconda3/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "/home/zing/anaconda3/etc/profile.d/conda.sh" ]; then
        . "/home/zing/anaconda3/etc/profile.d/conda.sh"
    else
        export PATH="/home/zing/anaconda3/bin:$PATH"
    fi
fi
```

2. Be careful to modify thr `.rviz` file in `tesseract`or it may cause a `md5sum` error.

```bash
[ERROR] [1695216596.972601504]: Client [/kortex_motion_planning_rviz] wants topic /gen3_environment/tesseract_published_environment to have datatype/md5sum [tesseract_msgs/Environment/d72de78ba1a2e7ee13dad5cb397289bb], but our version has [tesseract_msgs/EnvironmentState/a9ab9abd8193d6bd360758814884c19e]. Dropping connection.
```

3. Build `tesseract` implemented with a new version of `gazebo`

Install `gz-math7-dev`, `gz-common5-dev`, `gz-rendering7-dev` manually after running the script for updating the gazebo version:

```bash
#!/bin/bash
sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
apt-get update
```

which can be found at this [repo](https://github.com/tesseract-robotics/tesseract_qt/blob/main/.add-gazebo-ppa).

### AnyGrasp

Most of the building issues are due to version conflicts among packages. 

#### Build steps

##### General requirements

Those combinations are tested.

- `cuda11.0`,  `python 3.6`with trash low performance GPU `GeForce 1660Ti`
- `cuda11.0`, `python 3.8`, with trash low performance GPU `GeForce 1660Ti`
- `cuda11.1`, `python 3.8`, with `RTX A2000 Laptop`
- `cuda11.1`, `python 3.8`, with `GeForce 3050Ti`

Please note that using AnyGrasp for grasp generation consumes approximately 1.6GB of GPU memory, as tested on an RTX A2000 with 4GB of memory. You can decrease the resolution of the screen or close VScode, Typora, Google Chrome, etc,. to reduce the usage of GPU.

Note: cuda11.1 is cuda 11.1.105

##### Shortcut

One can reproduce a conda venv taking a shortcut. In the source device, run

```bash
conda activate anygrasp
conda env export > anygrasp.yml
```

Then in the target device, run

```
conda env create -f anygrasp.yml
conda activate anygrasp
```

In this way you will create a pre-configured virtual environment, ensuring no version conflicts occur. Notw that in this way, you may meet a pip error, this is normal, just ignore it and then you can start from step 3 in [Normal operation](#####Normal operation). One can find the available `yml` files at [venv](https://github.com/zingff/anygrasp_sdk/tree/master/grasp_generation/config/venv). 

Alternatively, you can create a new virtual environment following [Normal operation](#####Normal operation).

##### Normal operation

0. CUDA: make sure you have installed `CUDA 11.0` or `CUDA 11.1` correctly, one can check this by

   ```
   nvcc -V
   ```

   Other `CUDA` version might be available, but not tested yet.

1. venv: create a new conda virtual environment.

   ```bash
   conda create -n py3-mink python=3.8
   conda activate py3-mink
   ```

2. [MinkowskiEngine](https://github.com/NVIDIA/MinkowskiEngine): please follow the [official tutorial](https://github.com/NVIDIA/MinkowskiEngine). One recommended method is via [Anaconda](https://github.com/NVIDIA/MinkowskiEngine#cuda-11x) with CUDA 11.X. Note that you need to modify the version of `pytorch`, `cudatoolkit`, etc,. to desired values.

   - `CUDA` and `openblas`: 

     ```bash
     conda install openblas-devel -c anaconda
     conda install pytorch=1.9.0 torchvision cudatoolkit=11.1 -c pytorch -c nvidia
     ```

     please note the conda install order: `cudatoolkit`, then `openblas`. if 

     ```bash
     conda install pytorch=1.9.0 torchvision cudatoolkit=11.1 -c pytorch -c nvidia
     conda install openblas-devel -c anaconda
     ```

   - Local MinkowskiEngine (or use `pip install` method)

     ```bash
     export CUDA_HOME=/usr/local/cuda-11.1
     git clone https://github.com/NVIDIA/MinkowskiEngine.git
     cd MinkowskiEngine
     python setup.py install --blas_include_dirs=${CONDA_PREFIX}/include --blas=openblas
     
     ```

3. [anygrasp_sdk](https://github.com/graspnet/anygrasp_sdk): just follow the [official tutorials](https://github.com/graspnet/anygrasp_sdk#installation) to install it. Potential issues are listed in [Issues](###Issues).

4. `anygrasp_generation` setup: this is a custom package used to generate grasp for objects utilizing `RGBD` streams provided by Kinova Gen3. Please review the following key points before you running a demo:

   - `python` version: `gsnet.so` and `lib_cxx.so` are consistent with the venv python version.
   - `license`: this dir should be placed correctly under `anygrasp_sdk/grasp_generation/`
   - checkpoint file `log` should be placed under : `anygrasp_sdk/grasp_generation/`

#### Issues

1. `numpy` error: try the two sets of configuration which are tested working correctly.

   - `cuda11.0`, `python 3.6`, `numpy 1.19.2`, GeForce 1660Ti

   - `cuda11.0`, `python 3.8`, `numpy 1.23.4`, GeForce 1660Ti

   - `cuda11.1`, `python 3.8`, `numpy 1.23.4`, RTX A2000

   you can just simply set version of `numpy` by replacing `numpy` with `numpy==1.23.4` in [`anygrasp_sdk/requirements.txt`](https://github.com/graspnet/anygrasp_sdk/blob/main/requirements.txt). 

   ```
   numpy==1.23.4 # numpy==1.19.2
   Pillow
   scipy
   tqdm
   graspnetAPI
   open3d
   ```

   Issues about `numpy` might cause something like:

   ```bash
   $ sh demo.sh 
   RuntimeError: module compiled against API version 0x10 but this version of numpy is 0xd
   AttributeError: FvrLicenseWrapper
   
   The above exception was the direct cause of the following exception:
   
   ImportError: initialization failed
   ```

   ```bash
   $ sh demo.sh 
   AttributeError: module 'numpy' has no attribute 'float'.
   `np.float` was a deprecated alias for the builtin `float`. To avoid this error in existing code, use `float` by itself. Doing this will not modify any behavior and is safe. If you specifically wanted the numpy scalar type, use `np.float64` here.
   The aliases was originally deprecated in NumPy 1.20; for more details and guidance see the original release note at:
       https://numpy.org/devdocs/release/1.20.0-notes.html#deprecations
   ```

2. `sklearn` error: refer to [this issue](https://github.com/graspnet/anygrasp_sdk/issues/7). Note that after running

   ```bash
   pip install scikit-learn
   pip install open3d
   ```

   you need to run

   ```
   pip install -r requirements.txt
   ```

   again to ensure that all requirements are satisfied. This procedure may be executed several times. 

   Note: error may permanently occur when you run `pip install -r requirements.txt`, so you can try manually install `scikit-learn`, `open3d`, `graspnetAPI`, etc,. in `requirenments.txt`. Just ignore the error after running `pip install -r requirements.txt`.

3. Unknown error: try running

   ```bash
   export CUDA_HOME=/usr/local/cuda-11.1
   ```

   in your terminal before you build `MinkowskiEngine` or `anygrasp_sdk`.

4. Memory issues

   ```bash
   RuntimeError: CUDA error: out of memory
   ```

   or (not very sure, but it should be)

   ```bash
   Error from AnyGrasp server:  CUDA error: CUBLAS_STATUS_NOT_INITIALIZED when calling `cublasCreate(handle)`
   ```

   or

   ```bash
   Error from AnyGrasp server:  CUDA out of memory. Tried to allocate 28.00 MiB (GPU 0; 3.78 GiB total capacity; 319.96 MiB already allocated; 91.00 MiB free; 334.00 MiB reserved in total by PyTorch)
   ```

### Pinocchio

Issue:

```bash
CMake Error at /home/zing/mealAssistiveRobot/sla_ws/src/pinocchio/cmake/package-config.cmake:110 (find_package):
  By not providing "Findeigenpy.cmake" in CMAKE_MODULE_PATH this project has
  asked CMake to find a package configuration file provided by "eigenpy", but
  CMake did not find one.

  Could not find a package configuration file provided by "eigenpy"
  (requested version 2.7.10) with any of the following names:

    eigenpyConfig.cmake
    eigenpy-config.cmake

  Add the installation prefix of "eigenpy" to CMAKE_PREFIX_PATH or set
  "eigenpy_DIR" to a directory containing one of the above files.  If
  "eigenpy" provides a separate development package or SDK, be sure it has
  been installed.
Call Stack (most recent call first):
  CMakeLists.txt:167 (ADD_PROJECT_DEPENDENCY)
```

Try: install `ros-noetic-eigenpy` or follow this [link](https://stack-of-tasks.github.io/pinocchio/download.html).

### Eigen

```bash
sudo apt install libeigen3-dev
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
```



### Some links

- [**apriltag_ros**](https://github.com/AprilRobotics/apriltag_ros)
- [**Tesseract**](https://tesseract-docs.readthedocs.io/en/latest/_source/intro/getting_started_doc.html)
- [**Pinocchio**](https://github.com/stack-of-tasks/pinocchio)

# GPT-SAM Grasping

## Issues: 

1. for create the conda env with voice module:

```bash
Pip subprocess error:
  error: subprocess-exited-with-error
  
  × Building wheel for pyaudio (pyproject.toml) did not run successfully.
  │ exit code: 1
  ╰─> [18 lines of output]
      running bdist_wheel
      running build
      running build_py
      creating build
      creating build/lib.linux-x86_64-cpython-38
      creating build/lib.linux-x86_64-cpython-38/pyaudio
      copying src/pyaudio/__init__.py -> build/lib.linux-x86_64-cpython-38/pyaudio
      running build_ext
      building 'pyaudio._portaudio' extension
      creating build/temp.linux-x86_64-cpython-38
      creating build/temp.linux-x86_64-cpython-38/src
      creating build/temp.linux-x86_64-cpython-38/src/pyaudio
      gcc -pthread -B /home/zing/anaconda3/envs/ai/compiler_compat -Wl,--sysroot=/ -Wsign-compare -DNDEBUG -g -fwrapv -O3 -Wall -Wstrict-prototypes -fPIC -I/usr/local/include -I/usr/include -I/home/zing/anaconda3/envs/ai/include/python3.8 -c src/pyaudio/device_api.c -o build/temp.linux-x86_64-cpython-38/src/pyaudio/device_api.o
      src/pyaudio/device_api.c:9:10: fatal error: portaudio.h: No such file or directory
          9 | #include "portaudio.h"
            |          ^~~~~~~~~~~~~
      compilation terminated.
      error: command '/usr/bin/gcc' failed with exit code 1
      [end of output]
  
  note: This error originates from a subprocess, and is likely not a problem with pip.
  ERROR: Failed building wheel for pyaudio
ERROR: Could not build wheels for pyaudio, which is required to install pyproject.toml-based projects

failed

CondaEnvException: Pip failed
```

try:

```bash
sudo apt install portaudio19-dev 
```

2. In a new venv run `yolo_sam_class`:

```bash
(sam) zing@zing-p15:~/mealAssistiveRobot/sla_ws/src/Grounded-Segment-Anything/yolo_sam/src$ python yolo_sam_class.py 
loading Roboflow workspace...
loading Roboflow project...
ROS Initialization done
Processing image...
Traceback (most recent call last):
  File "yolo_sam_class.py", line 251, in <module>
    processor.process_image()
  File "yolo_sam_class.py", line 78, in process_image
    image_np = self.bridge.imgmsg_to_cv2(self.raw_img, desired_encoding="bgr8")
  File "/opt/ros/noetic/lib/python3/dist-packages/cv_bridge/core.py", line 163, in imgmsg_to_cv2
    dtype, n_channels = self.encoding_to_dtype_with_channels(img_msg.encoding)
  File "/opt/ros/noetic/lib/python3/dist-packages/cv_bridge/core.py", line 99, in encoding_to_dtype_with_channels
    return self.cvtype2_to_dtype_with_channels(self.encoding_to_cvtype2(encoding))
  File "/opt/ros/noetic/lib/python3/dist-packages/cv_bridge/core.py", line 91, in encoding_to_cvtype2
    from cv_bridge.boost.cv_bridge_boost import getCvType
ImportError: /lib/x86_64-linux-gnu/libp11-kit.so.0: undefined symbol: ffi_type_pointer, version LIBFFI_BASE_7.0

```

try:

```
rm ${CONDA_PREFIX}/lib/libffi.7.so ${CONDA_PREFIX}/lib/libffi.so.7 
```

The problem is that those libs are dynamically linked to libffi8, which is incorrect. See [link](https://stackoverflow.com/questions/75045632/apt-update-failed-libp11-kit-so-0-undefined-symbol-ffi-type-pointer-version).

# Packages

## Robot control

### [Orocos (Open Robot Control Software)](https://orocos.org/)

## Planning algorithms

### [OMPL (the Open Motion Planning Library)](https://ompl.kavrakilab.org/)

**OMPL**, the Open Motion Planning Library, consists of many state-of-the-art *sampling-based motion planning algorithms*. OMPL itself does not contain any code related to, e.g., collision checking or visualization. This is a deliberate design choice, so that OMPL is not tied to a particular collision checker or visualization front end. The library is designed so it can be easily integrated into [systems that provide the additional needed components](https://ompl.kavrakilab.org/integration.html).

### [TrajOpt (Trajectory Optimization for Motion Planning)](https://rll.berkeley.edu/trajopt/doc/sphinx_build/html/ )

*trajopt* is a software framework for generating robot trajectories by local optimization.

The following core capabilities are included:

- a solver for non-convex optimization problems, using sequential convex optimization.
- cost and constraint functions for kinematics and collision avoidance
- constructing problems from JSON-based specification format

The core libraries are implemented in C++ ([API docs](https://rll.berkeley.edu/trajopt/doc/dox_build/index.html)), and python bindings are generated using boost python.

The theory and technical details of this software are described in a paper [Finding Locally Optimal, Collision-Free Trajectories with Sequential Convex Optimization](http://www.eecs.berkeley.edu/~joschu/docs/trajopt-paper.pdf).

Source code is available on [github](https://github.com/joschu/trajopt).

See application implemented via [MoveIt](https://ros-planning.github.io/moveit_tutorials/doc/trajopt_planner/trajopt_planner_tutorial.html).

Consider the non-convex optimization problem:

<img src="https://ros-planning.github.io/moveit_tutorials/_images/non_convex.png" alt="../../_images/non_convex.png" style="zoom:50%;" />

where *f(x)* is the minimum-length path given by

<img src="https://ros-planning.github.io/moveit_tutorials/_images/fx.png" alt="../../_images/fx.png" style="zoom: 50%;" />

The basic idea is to iterate by maintaining an estimate of the solution and a convex trust region over which we trust our solution. The two key points are:

- convex approximation of *f(x)* and *g(x)* over the trust region. Then the approximated ones are converted to penalty functions.
- affine approximation of *h(x)* over the trust region. Then the approximated *h(x)* is convetered to penalty function by considering its absolute value.

The following figure shows how TrajOpt algorithm works

<img src="https://ros-planning.github.io/moveit_tutorials/_images/algorithm.png" alt="../../_images/algorithm.png" style="zoom: 50%;" />

The bold italic parameters are the ones loaded from yaml file. All the constraints including obstacle avoidance, joint limits and target poses in joint space or Cartesian space are converted to penalty functions so they make the final objective function. The most inner loop is where sequential quadratic programming is used as a trust region method. It calculates the second-order Taylor approximation in a box trust region.

One important part of TrajOpt is how the obstacle avoidance constraint is formulated. In discrete case, the constraint is basically the difference between the signed distance (between robot link with itself or with obstacles) and a safe value and in the continuous case, the signed distance is between convex hull of two waypoints and obstacles.

## Collision

### [FCL](https://github.com/flexible-collision-library/fcl)

FCL is a library for performing three types of proximity queries on a pair of geometric models composed of triangles.

- Collision detection: detecting whether the two models overlap, and optionally, all of the triangles that overlap.
- Distance computation: computing the minimum distance between a pair of models, i.e., the distance between the closest pair of points.
- Tolerance verification: determining whether two models are closer or farther than a tolerance distance.
- Continuous collision detection: detecting whether the two moving models overlap during the movement, and optionally, the time of contact.
- Contact information: for collision detection and continuous collision detection, the contact information (including contact normals and contact points) can be returned optionally.

###  [OctoMap library](https://octomap.github.io/)

The [OctoMap library](https://octomap.github.io/) implements a 3D occupancy grid mapping approach, providing data structures and mapping algorithms in C++ particularly suited for robotics. The map implementation is based on an octree and is designed to meet the following requirements:

- **Full 3D model.** The map is able to model arbitrary environments without prior assumptions about it. The representation models occupied areas as well as free space. Unknown areas of the environment are implicitly encoded in the map. While the distinction between free and occupied space is essential for safe robot navigation, information about unknown areas is important, e.g., for autonomous exploration of an environment.
- **Updatable.** It is possible to add new information or sensor readings at any time. Modeling and updating is done in a *probabilistic* fashion. This accounts for sensor noise or measurements which result from dynamic changes in the environment, e.g., because of dynamic objects. Furthermore, multiple robots are able to contribute to the same map and a previously recorded map is extendable when new areas are explored.
- **Flexible.** The extent of the map does not have to be known in advance. Instead, the map is dynamically expanded as needed. The map is multi-resolution so that, for instance, a high-level planner is able to use a coarse map, while a local planner may operate using a fine resolution. This also allows for efficient visualizations which scale from coarse overviews to detailed close-up views.
- **Compact.** The map is stored efficiently, both in memory and on disk. It is possible to generate compressed files for later usage or convenient exchange between robots even under bandwidth constraints.

### [PQP(A Proximity Query Package)](http://gamma.cs.unc.edu/SSV/)

PQP is a library for performing three types of proximity queries on a pair of geometric models composed of triangles:

- *Collision detection* - detecting whether the two models overlap, and optionally, all of the triangles that overlap.
- *Distance computation* - computing the minimum distance between a pair of models, i.e., the distance between the closest pair of points.
- *Tolerance verification* - determining whether two models are closer or farther than a tolerance distance.

## Optimization

### [IFOPT](http://wiki.ros.org/ifopt)

A modern, light-weight, [Eigen](http://eigen.tuxfamily.org/)-based C++ interface to Nonlinear Programming solvers, such as [Ipopt](https://projects.coin-or.org/Ipopt) and [Snopt](http://ampl.com/products/solvers/solvers-we-sell/snopt/).

### [The *OSQP* (Operator Splitting Quadratic Program)](https://github.com/osqp/osqp)

The *OSQP* (Operator Splitting Quadratic Program) solver is a numerical optimization package for solving problems in the form.

# Tesseract and TrajOpt

## Workflow for trajectory planning

> basic_cartesian_example.cpp
>
> glass_upright_example.cpp

This part will state the general steps showing in the examples to create and solve a planning problem.

### Create monitor

Monitor is defined for visualization in rviz. You need to modify some config parameters:

1. launch file

```xml
    <node if="$(arg rviz)" pkg="rviz" type="rviz" name="kortex_motion_planning_rviz"
        args="-d $(find kortex_motion_planning)/config/kortex_motion_planning.rviz" />
```

2. rviz file

```
Monitor Topic: /gen3_environment/tesseract_published_environment
tesseract::EnvMonitorTopic: /gen3_environment/tesseract_published_environment
```

Note that the key point is to change the tesseract environment namespace that is nominated as `gen3_environment` above in both 

- `Class: tesseract_rviz/TesseractWorkbench/Environment Properties/Monitor Topic`: seems unnecessary.
- `Class: tesseract_rviz/tesseract::EnvMonitorTopic`

### create environment

> [Tesseract Environment](https://tesseract-docs.readthedocs.io/en/latest/_source/core/overview/index.html )

The Tesseract Environment is a manager which interface with the State Solver, Scene Graph, Contact Managers, Manipulator Manager and Command History.

#### Tesseract Kinematics Package

> [Tesseract Kinematics Package](https://github.com/tesseract-robotics/tesseract_docs/blob/3cd485a843e521cb2c2344110d332ea69405f375/_source/core/packages/tesseract_kinematics_doc.rst)

This package contains a common interface for Forward and Inverse kinematics for Chain, Trees and Graphs including implementation using KDL and OPW Kinematics. Note that the kinematics are load as plugins through a yaml config file which is added to the SRDF file.

More details see the document for [Tesseract Kinematics Package](https://github.com/tesseract-robotics/tesseract_docs/blob/3cd485a843e521cb2c2344110d332ea69405f375/_source/core/packages/tesseract_kinematics_doc.rst).

#### Tesseract SRDF Format

> [Tesseract SRDF Format](https://github.com/tesseract-robotics/tesseract_docs/blob/3cd485a843e521cb2c2344110d332ea69405f375/_source/core/packages/tesseract_srdf_doc.rst)

Tesseract has its own SRDF format which is similar to the one used through ROS, but includes features specific to Tesseract.

Two methods for creatine user-defined SRDF:

- See [Tesseract SRDF Format](https://github.com/tesseract-robotics/tesseract_docs/blob/3cd485a843e521cb2c2344110d332ea69405f375/_source/core/packages/tesseract_srdf_doc.rst) for a text-editting method (not recommended but maybe the currently unique way).
- See [Tesseract Setup Wizard](https://tesseract-docs.readthedocs.io/en/latest/_source/intro/tesseract_setup_wizard_doc.html) for a GUI method (recommended but works within `tesseract-ignition` are transferring to a new package named `tesseract-studio`, maybe).

### create visualization for collision objects

### create task composer plugin factory

### create program

### set move instruction for the program

#### create waypoints

##### CartesianWaypoint

This is a Cartesian pose leveraged within the Plan Instruction.

##### JointWaypoint

This is a joint space pose leveraged within the Plan Instruction.

##### StateWaypoint

This not only includes joint names and positions but also includes velocity, acceleration, and time from start. It is primarily used in the Move Instruction but may be used in the Plan Instruction.

##### Workflow for creating waypoints

- set the robot initial state
- set the robot target state

#### create move instructions for program

##### MoveInstructionType

> [PlanInstructionType](https://github.com/tesseract-robotics/tesseract_planning/issues/101 )

MoveInstructionType detemines how the motion planner input (i.e. Cartesian poses) gets interpolated to a more dense input by the `SimplePlanner` for use by the other motion planners.

- MoveInstructionType::LINEAR

New poses are generated in a `straight line` between two waypoints.

- MoveInstrutionType::FREESPACE

Waypoints are generated along a `joint interpolated arc` between two waypoints.

Both strategies (Linear and Freespace) are valid for all combinations of joint and Cartesian waypoints (i.e. joint to joint, joint to Cartesian, Cartesian to joint, and Cartesian to Cartesian) and IK/FK are used to get the waypoints into the correct state space.

- MoveInstructionType::CIRCULAR

- [ ] Usage



#### add instructions to program

### print diagnotics

### create executor

### create profile dictionary

Every *cost* or *constraint* will get converted to a `quadratic function`:
$$
e(x) = A+Bx+Cx^2
$$
and the coefficient is applied as follow:
$$
e(x) = WA+(WB)x + (WC)x^2
$$
Usually, the `weight/scale` is suggested to set between 1~20, but can go higher. But if you start seeing solver failures you probably have gone too high.

#### Tesseract Planner Package

> [Tesseract Planner Package](https://github.com/tesseract-robotics/tesseract_docs/blob/3cd485a843e521cb2c2344110d332ea69405f375/_source/core/packages/tesseract_planners_doc.rst)

This package contains a common interface for Planners and includes implementation for OMPL, TrajOpt and Descartes.

#### composite profile

The composite profile is related to *a collection of waypoints*.

#### Collision terms (within trajopt)

> [collision terms](https://github.com/tesseract-robotics/trajopt/discussions/258 )

##### CollisionEvaluatorType

The CollisionEvaluatorType is trajopt::CollisionEvaluatorType.

- DISCRETE_CONTINUOUS = 1 - This perform discrete collision check at intermediate states between two trajectory state. The number of interpolated state is determined by the longest valid segment fraction. It simply samples the trajectory and doing discrete checks.
- CAST_CONTINUOUS = 2 - This does the same interpolate as DISCRETE_CONTINUOUS but it creates a casted collision object between the interpolated states. Casted means continuous collision check which is elaborated in Chaper V in [TrajOpt paper](https://journals.sagepub.com/doi/abs/10.1177/0278364914528132?journalCode=ijra).
- SINGLE_TIMESTEP = 0- This only check collision at the trajectory states

##### safety_margin

The safety_margin is used to calculate error for cost or constraint. The safety_margin_buffer is used when calculating the jacobian and error used for the quadratic function. This allows it to even though you may not be violating a constraint to still add a quadratic function to the problem so it understands the direction that will cause it to move closer to the violation. Ideally you would want to set the contact distance to infinite but this would come at a computation cost so this allows to you *set the contact distance threshold* used which is *safety_margin + safety_margin_buffer*.

Adding it as a constraint for convoluted scenarios can be problematic because it is a local optimal planner and at least for the open source solvers it does not allow it move away increasing the constraint violation and then come back to new local minimum. Though when adding them as a cost the solvers allows it to do this. We solve this two ways. The first is to leverage another planner to create a better seed like descartes or ompl. The second is to run a two stage trajopt but running the collision with cost and then after it finish solving running it using constraints. I have thought about update trajopt solver to support this. Also the commercial solver seem to handle these condition better.

- It is a proper way to set the safety_margin_buffer = 2$\times$ safety_margin. Non-zero safety buffer will allow the solving process of optimization more stable and make it converge faster.

- Difference between `collision_constraint_config` and `collision_cost_config`

If you only set the collision as a cost, it is not guaranteed to be collision-free because the solver will just try to minimize it as best it can. If it is a constraint the solver will fail if any contact is within the safety margin. 

Now you may be wondering then why would you add it as a cost. In some complicated environments running the solver first with a collision as *cost* followed by running it again as a *constraint* performs better. Where if you added it as a constraint to start it may just fail because the initial state is in the infeasible region. I have also had good luck adding collision as both a cost and constraint with different safety margin distances.

#### Collision in TrajOpt and TrajOptIfopt

> [Collision in TrajOpt and TrajOptIfopt](https://github.com/tesseract-robotics/tesseract_planning/issues/147)

There is a fundamental difference between TrajOpt and TrajOptIfopt when it comes to how collision are added to the SQP problem. 

- TrajOpt

It adds every contact as a single equation which results in a large number of equations added.

- TrajOptIfopt

The number of equations cannot be dynamic so we had to change things. If you set `num_eq = 3`,  it will find the 3 pairs of links which have the worst contact distance and generate a weighted average for gradient for a given link pair and use the worst contact distance as the constant when it creates the convex approximation. 

This appears to make TrajOptIfopt more stable when dealing with octomaps because for a given link pair that includes octomap you get a single equation that represents all of the contacts.

#### plan profile

The plan profile is specific to each waypoint.

##### cartesian_coeff

> [cartesian_coeff_1](https://github.com/tesseract-robotics/tesseract_planning/discussions/84 )
>
> [cartesian_coeff_2](https://github.com/tesseract-robotics/tesseract_ros/discussions/188 )

`cartesian_coeff` are the weights for **gradient descent** with which the optimization will try to satisfy Cartesian position (pose? to fix) constraints for each dimension of the space `(x, y, z, rx, ry, rz)`. 

- **<u>The larger the weights, the more focus will be put on reducing errors for the quantities associated with those weights.</u>** 
- <u>Usually all are set to the *same value*, so the optimization tries to satisfy all elements of the space with equal strength.</u> 
- <u>**When a value is set to zero, it effectively allows the optimization to ignore errors in that dimension.**</u> For example, if the weights are `[10, 10, 10, 10, 10, 0]`, then the optimization will unconstrain the rotation about `rz` until the other five dimension goals are satisfied. 
- <u>Similarly a weight vector of `[0, 0, 0, 10, 10, 10]` will allow the position to float freely while meeting orientation constraints.</u>

The size of `cartesian_coeff` should be 6. The first three correspond to translation and the last three correspond to rotation. For example, `plan_profile->cartesian_coeff(5) = 0` indicates it is free to rotate around the z-axis. If the value is greater than zero it is considered constrained and the coefficient represents a *weight/scale* applied to the error and gradient.

In `glass_upright_example`, the last three `cartesian_coeff` are set to be 5 while the first three ones are 0, which indicates the end effector of the manipulator can *translate* freely  but is constrained to *rotate*. The result of the optimization for a *UPRIGHT* goal is that the end effector goes a semi-circle trajectory. 

Chaning the tranlation coefficients to 0.1, 1, 5 and 20 will lead the planning trajectory being a straight line go throuh the sphere. To 200 the result holds.

 <img src="/home/zing/Documents/notes/researchNote.assets/Screenshot 2023-05-18 161826.png" alt="Screenshot 2023-05-18 16:18:26" style="zoom: 33%;" />

This is because the `coeff for the cartesian position` is significantly higher than the one for the collision so it dominates the optimization. Warning log: 

```
[WARN] approximate merit function got worse (-5.869e-04). (convexification is probably wrong to zeroth order)
[WARN] approximate merit function got worse (-7.315e-02). (convexification is probably wrong to zeroth order)
[WARN] approximate merit function got worse (-5.913e-01). (convexification is probably wrong to zeroth order)
[WARN] approximate merit function got worse (-5.223e+00). (convexification is probably wrong to zeroth order)
[WARN] convex solver failed! set TRAJOPT_LOG_THRESH=DEBUG to see solver output. saving model to /tmp/fail.lp and IIS to /tmp/fail.ilp
```

- [ ] Since the cartesian position is limited (first three coefficients are all greater than 0) , why the optimization still have solution (a straight line trajectory) ?

##### joint_coeff

Similar to the `cartesian_coeff`, the size of `joint_coeff` is identical to the number of `dof` of a manipulator. If a certain `cartesian_coeff` is set to be 0, then the corresponding joint is free to apply any rotational movement. 

TODO: 

- [ ] To verify the guess.

#### add profile to dictionary

### create task composer

> [Tesseract Task Composer](https://github.com/tesseract-robotics/tesseract_planning/tree/master/tesseract_task_composer)

#### Tesseract Task Composer

An interface for general purpose *parallel task programming* system.

- TaskComposerExecutor - A interface for *executing* a task or graph of task
- TaskComposerNode - A interface for *implementing* an task or graph
- TaskComposerServer - Loads a yaml config which import TaskComposerExecutor and TaskComposerNode plugins.

#### Task Composer Plugin Config

This file allows you to define Executors and Tasks (a.k.a. Nodes). Note that Not all nodes are intendend to be standalone task but be comsumed by a task.

See [task_composer_plugins.yaml](https://github.com/tesseract-robotics/tesseract_planning/blob/master/tesseract_task_composer/config/task_composer_plugins.yaml)

#### Workflow to create a task composer

- create executor

This should write before [create profile dictionary](###create profile dictionary).

```c++
  auto executor = factory.createTaskComposerExecutor("TaskflowExecutor");
```

- create task

```c++
  const std::string task_name = (ifopt_) ? "TrajOptIfoptPipeline" : "TrajOptPipeline";
  TaskComposerNode::UPtr task = factory.createTaskComposerNode(task_name);
  const std::string input_key = task->getInputKeys().front();
  const std::string output_key = task->getOutputKeys().front();
```

- create task input data

```c++
  TaskComposerDataStorage input_data;
  input_data.setData(input_key, program);
```

- create task composer problem

```c++
  TaskComposerProblem problem(env_, input_data);
```

### slove task

#### Planners

> [TrajOptTaskflow](https://github.com/tesseract-robotics/tesseract_planning/issues/101 )

##### SimplePlanner

The *default* `SimplePlanner` (which is a part of most planning taskflows, i.e. process planners) performs interpolation based on these strategies using a "longest-valid-segment" technique, meaning it adds an interpolated point a some discrete Cartesian or joint distance (e.g. every 25mm or every 1 degree of joint space movement). That's why your video shows many Cartesian waypoints when you only actually gave the planner 2 waypoints. There are other `SimplePlanner` profiles that have different behavior, such as copying the first waypoint n times, interpolating over n states, etc.

##### TrajOpt planner

It is *the only* planner that is capable of moving these interpolated waypoints out of collision. 

##### OMPL planner

It does not require interpolation because it generates *unconstrained collision-free intermediate states* between the start and goal waypoints as a part of the planning process.

##### Decartes planner

It usually benefits from interpolation, but only samples valid robot poses at each interpolated waypoint and cannot move that waypoint out of collision.

#### Pipeline (Planning taskflow, process planner)

This mainly couples with *interpolation* process. See [task_composer_plugins.yaml](https://github.com/tesseract-robotics/tesseract_planning/blob/master/tesseract_task_composer/config/task_composer_plugins.yaml)

##### FreespaceTaskflow (FREESPACE_PLANNER_NAME)

> SimplePlanner->TrajOpt->OMPL (->TrajOpt)

- For *freespace moves* between waypoints where it doesn't really matter what path the robot takes to get there
- Interpolates with a `SimplePlanner` (based on the `PlanInstructionType` described above)
- First attempts planning with the TrajOpt planner (which can, and probably will, adjust the positions of the interpolated waypoints)
- If TrajOpt fails, attempts planning with OMPL (disregarding the interpolated path from the `SimplePlanner`) to generate a new collision-free trajectory
- If OMPL succeeds, uses TrajOpt to improve the trajectory from OMPL (where TrajOpt can, and probably will, adjust the intermediate joint states of the trajectory)
  - <u>You can control the behavior of TrajOpt by defining your own TrajOpt profiles that get added to the *planning request*</u> and are referenced by your original waypoints

##### CartesianTaskflow (CARTESIAN_PLANNER_NAME)

> SimplePlanner->Decartes (->TrajOpt)

- For move between waypoints where the robot must move in a specific pre-defined way where *no adjustment to the path is allowed*. <u>Most useful for systems with more than 6 degrees of freedom or underconstrained applications (i.e. tool z orientation is not constrained)</u>
- Interpolates with a `SimplePlanner` (based on the `PlanInstructionType` described above)
  - Depending on how dense your initial tool path is, you may not actually want to interpolate it. In this case you would want to create a custom `SimplePlanner` profile that does not attempt interpolation or create a custom planning taskflow that does not include a simple planner
- Uses the Descartes planner to sample robot poses (i.e. discretely sampling extra degrees of freedom or poses about an unconstrained dimension) for each waypoint in the path and construct a valid joint trajectory through that path. Note <u>Descartes cannot adjust the positions of these waypoints to make them collision-free.</u> It can only choose robot configurations that achieve the waypoint position in a collision-free manner.
- Uses TrajOpt to improve the trajectory from Descartes
  - Again you can configure the behavior of TrajOpt by defining a custom profile and associating your original waypoints with those profiles. By default we don't allow TrajOpt to adjust the Cartesian positions of the waypoints in this planner, but we do allow it to move the extra degrees of freedom of the system (e.g. a rail or gantry) to produce a smoother trajectory

### plot process trajectory

### TODO

- [ ] Modify `urdf` and `srdf` file for Gen3.

- [ ] Add wheelchair collision to Tesseract environment using `stl` or `dae` files. See [smartwheelchair](https://github.com/yaxianfan/smart_wheelchair)

- [x] Initial state is given by joint waypoint/ state waypoint with `joint_names` and `joint_pos`.
- [x] Target state is given by cartesian waypoint.
- [ ] Can change the waypoint number.
- [ ] Get the current state of the real arm before we create a planning task.

## Examples info

### basic_cartesian_example.launch

13  more move instructions

### car_seat_example.launch

7 + 7

### freespace_hybrid_example.launch

8

### freespace_ompl_example.launch

8

### glass_upright_example.launch

7

### glass_upright_ompl_example.launch

error

### kortex_trajopt_server_test.launch

7  a mimic from glass_upright_example

### online_planning_example.launch

error

### pick_and_place_example.launch

11 + 13

### puzzle_piece_auxillary_axes_example.launch

437 + 437

### puzzle_piece_example.launch

437 + 437

# Kinova

## Gen3 connection instructions

General process to connect the robot via `Ethernet`:


  - Settings -> Network -> Wired -> `Setting icon` -> IPv4
  - Shift the `IPv4 Method` to `Manual`
  - Configurations:
    * Address: `192.168.1.6`, the port should aviod the arm's one, thus not be `192.168.1.1` or `192.168.1.10`(perhaps those two are the addresses of the robot and the web.)
    * Netmask: `255.255.255.0` or `24`
  - In browser, switch to `192.168.1.10` and you can assess the `KINOVA ® KORTEX™ Web App`.
  - If the connection is configured correctly, the Web application should launch and present a login window. Enter the following credentials:
    * username: admin
    * password: admin
  - Then you can monitor the robot status and play some pre-installed demos (home, zero, packaging and retract). 
    **NOTES** 
  - If you are using VPN on your ubuntu, remember to add `ignore hosts`:

    - `Settings` -> `Network` -> `VPN` -> `Network Proxy` -> `Manual` -> `Ignore Hosts`
    - Add  some hosts that you wish not to connect via proxy. For Kinvoa Gen3, you would add `192, 168, 1, *`. For campus website, you would add `*.sjtu.edu.cn`
  - You could click the `Hold` buttom to ensure the execution of those demos by a single click.
  - Do pay attention to your safety when excute those demos since there are no forque sensing in those processes.

## Angles conversion

All angles using by kinova are in *degree* metric. Cartesian pose of `tool_frame` feedbacked by `BaseCyclic_Feedback`:

```c++
ros::topic::waitForMessage<kortex_driver::BaseCyclic_Feedback>("/my_gen3/base_feedback");
```

is constructed in ZYX Euler angles (Tait-Bryan angles, yaw-pitch-roll). If you are manually converting a set of ZXY Euler angles to quaternion, you may follow the following order:

| Angles type | x          | y          | z          |
| :---------- | ---------- | :--------- | :--------- |
| ZYX         | $\theta_z$ | $\theta_y$ | $\theta_x$ |
| feedback    | $\theta_x$ | $\theta_y$ | $\theta_z$ |

For online converter, see [3D Rotation Converter](https://www.andre-gaschler.com/rotationconverter/).

## TCP speed limiter

This is implemented via Tesseract and the algorithm is given below.

1. Given the joint trajectory to be optimized.

   ```c++
   tesseract_planning::JointTrajectory output_trajectory = input_trajectory;
   ```

2. Create state solver to calculate the forward kinematics, i.e., from joint space to cartesian space, thus the TCP should be specified.

   ```c++
   tesseract_scene_graph::StateSolver::UPtr state_solver = env_->getStateSolver();
   ```

3. Find the adjacent waypoints that require the biggest speed reduction to stay under the maximum TCP speed which should be defined under the instructions in Kinova official tutorial.

   ```c++
   tesseract_scene_graph::SceneGraph previous_scene_state = state_solver->getState(output_trajectory[i - 1].joint_names, output_trajectory[i - 1].position);
   Eigen::Isometry3d previous_pose = previous_scene_state.link_transforms[tcp];
   ```

4. Calculate the average TCP velocity between the two adjacent waypoints.
   $$
   \begin{align} 
   & \forall \bold{\ i} \in \text{waypoint space} \\
   & \Delta x = || x_i - x_{i-1}|| \\
   & \Delta t = t_i - t_{i-1} \\
   & \bar{v} = \Delta x / \Delta t \\
   & \bold{if \ } \bar{v}>v_max\\
   & \alpha = v_max / \bar{v}\\
   & \bold{if \ } \alpha < \alpha_{max} = 1.0 \\
   & \alpha_{max} \gets \alpha \\
   & ...
   \end{align}
   $$

## Order in euler angles

[Roll pitch and yaw from Rotation matrix with Eigen Library](https://stackoverflow.com/questions/27508242/roll-pitch-and-yaw-from-rotation-matrix-with-eigen-library)

(2, 1, 0) in Eigen:

```c++
Vector3f ea = m.eulerAngles(2, 1, 0);
```

## Collision detection parameter tuning

Based on the search results, there are several papers that could be relevant to your inquiry about setting different threshold values for different joints in robotic manipulators for collision detection. However, direct access to these papers is restricted. Here's a summary of what I found:

1. **Adaptive Threshold for Robot Manipulator Collision Detection Using Fuzzy System**: This paper, published in SN Applied Sciences, discusses an adaptive threshold for robot manipulator collision detection using a fuzzy system. This could provide insights into adaptive and dynamic approaches to setting thresholds for collision detection【52†source】【53†source】.

2. **Method for Detecting Collision in Robot Manipulator and Reacting According to Collision Direction**: This research successfully identified the collision impact at joints and reacted according to the collision direction, which may offer valuable information on handling collision impacts at different joints【54†source】.

3. **Collision Detection and Identification for a Legged Manipulator**: This paper suggests a method for setting detection thresholds and discusses the phases following collision detection, including isolation and identification phases【55†source】.

4. **Collision Detection Algorithm for Manipulator Planning Process**: This research utilizes robot proprioceptive sensors (encoders and torque sensors) for collision detection, which might align with your current approach of using discrepancies between joint torque sensors and a dynamic model【57†source】.

Since direct access to the full papers is not available, I recommend looking up these papers in a university library or a digital library subscribed to these journals. The titles and authors provided should help you locate the papers for detailed study. Additionally, exploring these papers might offer novel insights or methodologies that you can adapt for setting joint-specific collision detection thresholds in robotic manipulators.

1. **Adaptive Threshold for Robot Manipulator Collision Detection Using Fuzzy System**:
   - Authors: Omar Abdelaziz, Minzhou Luo, Guanwu Jiang, & Saixuan Chen
   - Published in: SN Applied Sciences
   - Link: [Springer - Adaptive threshold for robot manipulator collision detection using fuzzy system](https://link.springer.com/article/10.1007/s42452-020-2110-z)

2. **Method for Detecting Collision in Robot Manipulator and Reacting According to Collision Direction**:
   - Excerpt from a paper on safe human–robot collaboration
   - Link: [Semantic Scholar - Adaptive threshold for robot manipulator collision detection](https://www.semanticscholar.org/paper/Adaptive-threshold-for-robot-manipulator-collision-Abdelaziz-Luo/5048f86e5a6161f4de29a071a9551684db646ae7)

3. **Collision Detection and Identification for a Legged Manipulator**:
   - Focuses on detection threshold and collision identification
   - Link: [ArXiv - Collision detection and identification for a legged manipulator](https://arxiv.org/abs/2002.05168)

4. **Collision Detection Algorithm for Manipulator Planning Process**:
   - Discusses the use of proprioceptive sensors for collision detection
   - Link: [ScienceDirect - Collision detection and identification for robot manipulators based on proprioceptive sensors](https://www.sciencedirect.com/science/article/abs/pii/S0967066118302466)

# Ubuntu

## **[MEMORY CONSUMPTION]** Speed build time when `catkin build` 

**Without** setting `swap` partition while installing `ubuntu`, the `swap` will have a default size of 2GB, which zone was allocated at `/swapfile`. When `catkin build` some projects, it is slow and sometimes sticks. Here is a way to allocate temp swap when you need to build it,

```bash
sudo mkdir /swap
cd /swap
sudo fallocate -l 16G swapfile
sudo chmod 600 /swap/swapfile
sudo mkswap /swap/swapfile  # start for each time
sudo swapon /swap/swapfile
sudo swapon --show
```

The swap returns to 2G once you reboot Ubuntu, so you should add swap size every time you need or set it permenently,

```bash
sudo cp /etc/fstab /etc/fstab.bak
echo ‘/swap/swapfile none swap sw 0 0’ | sudo tee -a /etc/fstab
```

## Conda

### auto-activate base:

```bash
conda config --set auto_activate_base false
```

## Git

### git global initialization

```bash
git config --global user.name "John Doe"
git config --global user.email "johndoe@email.com"
```

### libffi import error in conda environment

20231031

```
ImportError: /lib/x86_64-linux-gnu/libp11-kit.so.0: undefined symbol: ffi_type_pointer, version LIBFFI_BASE_7.0
```

[solution](https://blog.csdn.net/qq_38606680/article/details/129118491?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522169875515316800188551365%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=169875515316800188551365&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-129118491-null-null.142^v96^pc_search_result_base9&utm_term=ImportError%3A%20%2Flib%2Fx86_64-linux-gnu%2Flibp11-kit.so.0%3A%20undefined%20symbol%3A%20ffi_type_pointer%2C%20version%20LIBFFI_BASE_7.0&spm=1018.2226.3001.4187)

# Some libraries

## opencv

- version: 4.2.0 (deprecated)
- path: /usr/lib/python3/dist-packages/cv2.cpython-38-x86_64-linux-gnu.so (deprecated)

- CmakeLists modification: [official document](https://docs.opencv.org/4.x/db/df5/tutorial_linux_gcc_cmake.html)



# Others


## 20231001 ubuntu reinstall

cuda_11.0.3_450.51.06_linux.run
Anaconda3-2023.09-0-Linux-x86_64.sh
NVIDIA-Linux-x86_64-525.125.06.run
