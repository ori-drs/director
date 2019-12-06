#Director

![drc terrain segmentation](director_pic.png)

# Introduction

The Director is a interface for remote command and control of a field robot.

It is built within the ROS system and uses VTK for rendering.


System Requirements
-------------------

As of this writing, the software is tested on Ubuntu 14.04 and 16.04, and MacOSX 10.11.
The build should work on Microsoft Windows with MSVC but it is not continuously tested.
In theory it can run on any platform where VTK and Qt are supported.


Download Instructions
=====================

Install Git
-----------

The source code is stored in a Git repository. To download the
source code you may need to first install Git on your system.
On Mac, we recommend using Homebrew.  On Windows, download the
official git package from https://git-scm.com

Download the source code
------------------------

Download the repository with the ``git clone`` command:

::

    git clone https://github.com/RobotLocomotion/director.git


Dependencies
============


Required Dependencies
---------------------

The required 3rd party dependencies are:

  - Qt4 or Qt5 (Qt 4.8.7 recommended)
  - VTK 6.2+ (VTK 7.1.1 recommended)
  - Python 2.7 and NumPy

Additionally, you will need CMake 2.8 or greater to configure the source code.

The dependencies can be installed on Mac using `Homebrew <http://brew.sh/>`_:

::

    brew tap patmarion/director && brew tap-pin patmarion/director
    brew install cmake glib libyaml numpy python scipy vtk7
    pip2 install lxml PyYAML

The dependencies can be installed on Ubuntu using apt-get:

::

    sudo apt-get install build-essential cmake libglib2.0-dev libqt4-dev \
      libx11-dev libxext-dev libxt-dev python-dev python-lxml python-numpy \
      python-scipy python-yaml

On Ubuntu the build does not require VTK to be installed.  A compatible version
of VTK will be downloaded (precompiled binaries) at build time.


Building
========

Compiling
---------

::

    make superbuild

This is an alias for:

::

    mkdir build && cd build
    cmake ../distro/superbuild
    make


Documentation
=============

A preliminary Online Help for the Director (currently in preparation) can be found `here <https://openhumanoids.github.io/director/>`_.





# History

This repo (from Oxford Dynamic Robot Systems Group) is a fork of the original Director repo. Originally Director was developed as the primary user interface used by Team MIT in the DARPA Robotics Challenge.

[![Team MIT DRC day-1 visualization](https://img.youtube.com/vi/em69XtIEEAg/0.jpg)](https://www.youtube.com/watch?v=em69XtIEEAg)

` <https://www.youtube.com/watch?v=em69XtIEEAg>`_

This previous version was heavily integrated with Drake and did not use ROS. It continues to be developed as the user interface for Drake by Toyota Research Institute.

#Citing

If you wish to cite the director, please use:

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