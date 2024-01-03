# GPT-SAM Grasping

## Build instrctions

> last update: 20231219

### Package list (catkin list)

You can take a glimpse at the following list to know what each package is for.

1. *anygrasp_generation*
2. gazebo_version_helpers
3. gazebo_grasp_plugin
4. **geometry2**
5. *gpd_pick_and_place*
6. **kinova_vision**
7. kortex_control
8. **kortex_description**
9. gen3_lite_gen3_lite_2f_move_it_config
10. **gen3_move_it_config**
11. gen3_robotiq_2f_140_move_it_config
12. **gen3_robotiq_2f_85_move_it_config**
13. **kortex_driver***
14. kortex_examples
15. kortex_gazebo
16. **kortex_move_it_config**
17. roboticsgroup_upatras_gazebo_plugins
18. *kortex_motion_planning*
73. *yolo_sam*

### Build setup

1. Additional CMake Args: (not necessary, skip)
   `-DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.8 -DPYTHON_LIBRARY_DIR=/usr/lib/x86_64-linux-gnu`

2. install `python3-catkin-tools`

Other general steps for build a ROS catkin workspace (`init`, `rosdep`, `wstool` etc,.) are to be omitted in this tutorial.

### Recommended build procedure (update 20231220)

You can build the workspace in the following order. Or you can build the whole workspace one-off, but I never try.

1. ros_kortex

   gazebo_version_helpers gazebo_grasp_plugin kortex_control kortex_description gen3_lite_gen3_lite_2f_move_it_config gen3_move_it_config gen3_robotiq_2f_140_move_it_config gen3_robotiq_2f_85_move_it_config kortex_driver kortex_examples kortex_gazebo kortex_move_it_config roboticsgroup_upatras_gazebo_plugins

2. ros_kortex_vision

   kinova_vision

3. grasp generation

   anygrasp_generation

4. object_grasping 

   gpd_pick_and_place

5. rest (skip)
   segment_anything (deprecated)
   Grounded-Segment-Anything (python only)

## Troubleshooting

### Build steps and troubleshooting for AnyGrasp

Most of the building issues are due to version conflicts among packages. 

#### Build steps

##### General requirements

Those combinations are tested.

- `cuda11.0`,  `python 3.6`with `GeForce 1660Ti`
- `cuda11.0`, `python 3.8`, with `GeForce 1660Ti`
- `cuda11.1`, `python 3.8`, with `RTX A2000 Laptop`

Please note that using AnyGrasp for grasp generation consumes approximately 1.6GB of GPU memory, as tested on an RTX A2000 with 4GB of memory. You can decrease the resolution of the screen or close VScode, Typora, Google Chrome, etc,. to reduce the usage of GPU.

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

In this way you will create a pre-configured virtual environment, ensuring no version conflicts occur. Then you can start from step 3 in [Normal operation](#####Normal operation). Alternatively, you can set up the virtual environment on your own (just follow [Normal operation](#####Normal operation)). One can find the available `yml` files at [venv](https://github.com/zingff/anygrasp_sdk/tree/master/grasp_generation/config/venv).

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

3. [anygrasp_sdk](https://github.com/graspnet/anygrasp_sdk): just follow the official tutorials to install it. Possible issues are listed in [Issues](###Issues)
4. `anygrasp_generation` setup: this is a custom package used to generate grasp for objects utilizing `RGBD` streams from Kinova Gen3. Please review the following key points:
   
   - `python` version: `gsnet.so` and `lib_cxx.so` are consistent with the venv python version.
   - `license`: this dir should be placed correctly under `anygrasp_sdk/grasp_generation/`
   - checkpoint file `log` should be placed under : `anygrasp_sdk/grasp_generation/`

### Issues

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
   ```

   you need to run

   ```
       pip install -r requirements.txt
   ```

   again to ensure that all requirements are satisfied. This procedure may be executed several times. 

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

   

# 

### Troubleshooting for voice module

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

### Troubleshooting for sam

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

# Usage

Terminal 1: connect to the robot and start the RGBD streams

```bash
roslaunch kortex_driver gen3_connection.launch
```

Terminal 2: launch the anygrasp service for generating grasps of the objects

```bash
roslaunch anygrasp_generation object_grasp_detection.launch
```

Terminal 3: segment and recognize the object, then save the information (size, position) to a dictionary

```bash
python yolo_sam_class.py
```

Terminal 4: tell GPT what you would like to grasp, and then it will feedback you with a desired grasp sequence

```bash
python arm.py
```

Terminal 5: grasp the desired object

```bash
roslaunch gpd_pick_and_place anygrasp_multi_objects_grasp.launch
```



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

## Order in euler angles

[Roll pitch and yaw from Rotation matrix with Eigen Library](https://stackoverflow.com/questions/27508242/roll-pitch-and-yaw-from-rotation-matrix-with-eigen-library)

(2, 1, 0) in Eigen:

```c++
Vector3f ea = m.eulerAngles(2, 1, 0);
```

1. - https://www.sciencedirect.com/science/article/abs/pii/S0967066118302466)
