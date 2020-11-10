# sjinang/reinmav-gym

## Environment IDs
#### MujocoQuadReach-v0
#### MujocoQuadForest-v0

# Installation
## Requirements

- python3.6 (or 3.7) environment by one of the following 
    - system python 
    - conda 
    - virtualenv  
    - venv 
- [gym](https://github.com/openai/gym.git) 
- [vpython](https://vpython.org/)
- [baselines](https://github.com/openai/baselines.git) **SOURCE BUILD from 90d66776a49ad5e732b935dfc891bfbd06035ed2**
- matplotlib

## Notes

1. the code was tested on Ubuntu 16.04, 18.04 and macOS; but matplotlib has some issues in macOS. Please see [this doc](https://matplotlib.org/faq/osx_framework.html) for more details: we strongly recommend to use conda + pythonw (```conda install python.app```) on macOS.

2. pip package version of baselines has some issue. Please build from the source (commit code: [90d66776a49ad5e732b935dfc891bfbd06035ed2](https://github.com/openai/baselines/tree/90d66776a49ad5e732b935dfc891bfbd06035ed2))

## Install Dependencies

1. Install package dependencies
```sh
$ sudo apt update && sudo apt install libopenmpi-dev
```
2. Install gym. The installation guidelines can be found [here](https://gym.openai.com/docs/)
```sh
$ pip install gym
```
3. Install pip dependencies by 
```sh 
$ pip install -r requirements.txt
```
4. [Optional] Install mujoco, mujoco-py and gym[all]
    - See [For mujoco env (optional)](#for-mujoco-env-(optional)) for more details
5. [Optional] Install the baseline repository to use baseline algorithms to train the models
```
sudo apt-get update && sudo apt-get install cmake libopenmpi-dev python3-dev zlib1g-dev
git clone https://github.com/openai/baselines.git
cd baselines
pip install tensorflow-gpu # if you have a CUDA-compatible gpu and proper drivers
pip install -e .
```

## Installing the reinmav-gym package
1. Clone the package and cd into it
```
git clone https://github.com/sjinang/reinmav_gym.git
cd reinmav-gym
```
2. The environment is tested on python 3.6. Make sure you have the right python version when installing the environment
```
pip install -e .
```

## Check installation
You can check your installation using ```$ pip show```
```
pip show gym-reinmav
Name: gym-reinmav
Version: 0.0.1
Summary: UNKNOWN
Home-page: UNKNOWN
Author: UNKNOWN
Author-email: UNKNOWN
License: UNKNOWN
Location: /Users/YOUR_INSTALLED_PATH/openai/venv/lib/python3.6/site-packages
Requires: gym
Required-by: 
```

## For mujoco env (optional)

### Prerequisites

- [mujoco 1.5](http://www.mujoco.org/)
- [mujoco-py](https://github.com/openai/mujoco-py)

### Steps

1. download mjpro150 from [MuJoCo website](https://www.roboti.us/index.html)
2. put mjpro150 directory into ~/.mujoco
3. put mjkey.txt into ~/.mujoco
4. install apt dependencies
    - for example on Ubuntu 16.04:
    ```sh
    $ apt-get install -y python-pyglet python3-opengl zlib1g-dev libjpeg-dev patchelf \
        cmake swig libboost-all-dev libsdl2-dev libosmesa6-dev xvfb ffmpeg
    ```
    - see gym [README - Installing everything section](https://github.com/openai/gym#installing-everything) for more details.
5. export LD_LIBRARY_PATH
```sh
$ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.mujoco/mjpro150/bin
$ # check your nvidia driver version 
$ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia-<YOUR VERSION> 
```
6. install gym by 
```sh
$ pip3 install 'gym[all]'
```

Note. mujoco200 (MuJoCo 2.0) is not supported yet.


*****************************************************

SOME EXTRA IMPORTANT UPDATES, YOU WILL HAVE TO MAKE :

1. Make every env._entry_point -> env.entry_point in run.py 
2. Comment line 174 and 175 of run.py, both of which comes under  'if' for args.extra_import
3. Add \ at the end of line 96 in /envs/native/quadrotor2d.py

*****************************************************





