# Kortex Motion planning
Motion planning package for Kinova Gen3. This package implements algorithms from [Tesseract motion planning library](https://github.com/tesseract-robotics/tesseract_planning) for Kinova Gen3 7DOF. This package is utilized in both feeding task and object grasping with LLM.

## Basic structure

- [backup](backup): some backup files for file safety and reproduction.
- [config](config):
  - backup
  - data: directory to keep the realtime data as `csv` files
  - robot: directory to keep robotic arm model for collision detection, motion planning and trajectory optimization, etc,.
  - tesseract: stores config files such as task composer plugins and rviz file for solving and visualizing motion planning problems.
- [doc](doc): documents for all related packages
- other dirs: just as their names tell

## Tutorials

Note: All tutorials can be found in [doc](doc).

[General tutorial](doc/generalTutorial.md): installation steps of all packages used for feeding task, also including troubleshooting procedure for some specific and common issues occurring in tesseract and anygrasp setup.

## Dependencies
See [Tesseract Wiki](https://tesseract-docs.readthedocs.io/en/latest/_source/intro/getting_started_doc.html). 

## Build
See [Tesseract Wiki](https://tesseract-docs.readthedocs.io/en/latest/_source/intro/getting_started_doc.html).

## Others 
NONE