# Director

![director](director_pic.png)

# Introduction

The (Robot) Director is an interface for remote command and control of a field robot.

It is built within ROS and uses VTK for rendering. Unlike Rviz it is convenient to
adapt Director to be tightly integrated with a specific robot.

It's built upon a C++ base with the adaptation done in python - making rapid development
of autonomy easier.


System Requirements
-------------------

Currently the software is tested and used on Ubuntu 18.04 and ROS Melodic. ROS provides easy access to dependencies such as:

* PCL 1.8
* VTK 6.3
* OpenCV 3.2
* URDF/Xacro

The original upstream (see below) is compatible and tested with MacOSX 10.11. In theory it can run on any platform where VTK and Qt are supported including Windows.

# History

This repo (from Oxford Dynamic Robot Systems Group) is a fork of the original Director master. Originally Director was developed as the primary user interface used by Team MIT in the DARPA Robotics Challenge.

[![Team MIT DRC day-1 visualization](https://img.youtube.com/vi/em69XtIEEAg/0.jpg)](https://www.youtube.com/watch?v=em69XtIEEAg)

` <https://www.youtube.com/watch?v=em69XtIEEAg>`_

This previous version was heavily integrated with Drake and did not use ROS. It continues to be developed as the user interface for Drake by Toyota Research Institute.

#Citing

If you wish to cite the Director, please use this description from the DARPA Robotics Challenge:

::

    @article{2017JFR_marion,
      title = {Director: A User Interface Designed for Robot Operation With Shared Autonomy},
      author = {Pat Marion and Maurice Fallon and Robin Deits and Andr\'{e}s Valenzuela and
      Claudia P\'{e}rez D'Arpino and Greg Izatt and Lucas Manuelli and
      Matt Antone and Hongkai Dai and Twan Koolen and John Carter and
      Scott Kuindersma and Russ Tedrake},
      journal = "J. of Field Robotics",
      year = 2017,
      month = mar,
      volume = 34,
      issue = 2,
      pages = {225-426},
    }