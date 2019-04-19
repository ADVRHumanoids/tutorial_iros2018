# Tutorial IROS 2018
IROS 2018 Software Tutorial for XBotControl: refer to https://github.com/ADVRHumanoids/XBotControl for the software infrastructure.

## Where to start?

First of all you should follow the instruction here: https://github.com/ADVRHumanoids/XBotControl#install to install the latest version of XBotControl on your machine.

After this, you should clone and build the tutorial: there are several options to do it, but probably the easiest is using the catkin build tool from ROS.

```
mkdir -p ~/src/catkin_xbot_tutorial_ws/src
cd ~/src/catkin_xbot_tutorial_ws/src
catkin_init_workspace
git clone https://github.com/ADVRHumanoids/tutorial_iros2018.git
cd ..
catkin_make
```

Once you are done with the compilation, we should source the setup.bash of the catkin workspace just created:

```
. ~/src/catkin_xbot_tutorial_ws/devel/setup.bash
```
If you want to use the config file inside the tutorial workspace or your custom workspace, you should also export the XBOT_ROOT variable based on where your workspace is:

```
export XBOT_ROOT=~/src/catkin_xbot_tutorial_ws/src
```
All the relative path in the config files are going to be calculated based on XBOT_ROOT

## How to follow the tutorial?

A set of references you should always keep in mind are here: https://github.com/ADVRHumanoids/XBotControl/wiki  

The XBotControl tutorial tries to cover different components of the framework: you can follow it based on what are your need.

In each of the folder of the tutorial you will find a Jupiter notebook (.ipynb) with the instructions and insights related to the specific section.

Here it is how to install Jupyter Notebook on your pc: http://jupyter.org/install

Here it is the lists of the sections and where you will an answer to a set of possible questions:

- How do I control my robot using XBotControl -> **robots** folder and **configs** folder
- How do I write an RT plugin? -> **plugins** folder
- How do I find an example of an IK problem solved by OpenSoT whole-body IK engine? -> **opensot** folder
- How do I use the Cartesian Interface? -> **cartesio** folder

## Notes

Currently the master branch of this repo should not be used with the binary release v1.0.0: please refer to branch v1.0.0 for this.
