[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]

<br />
<p align="center">
  <a href="https://github.com/robotcraft19/robotcraft-pathfinding>
    <img src="res/images/logo_amazebot.png" alt="Logo" width="100" height="100">
    <img src="https://raw.githubusercontent.com/robotcraft19/robotcraft-pathfinding-stage/master/res/images/logo_amazebot.png">
  </a>

  <h3 align="center">Amazebot Real World Pathfinding Package</h3>

  <p align="center">
    Pathfinding package. Use A* to find optimal path in a mapped environment.
    <br />
    <a href="https://github.com/robotcraft19/robotcraft-pathfinding"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/robotcraft19/robotcraft-pathfinding">View Demo</a>
    ·
    <a href="https://github.com/robotcraft19/robotcraft-pathfinding/issues">Report Bug</a>
    ·
    <a href="https://github.com/robotcraft19/robotcraft-pathfinding/issues">Request Feature</a>
  </p>
</p>

## Table of Contents

* [About the Project](#about-the-project)
* [Setup](#setup)
* [Run](#run)
  * [Reactive](#reactive-navigation)
  * [Teleoperation](#teleoperation)
* [Roadmap](#roadmap)
* [Contribute](#contribute)
* [License](#license)
* [Contact](#contact)
* [Contributors](#contributors)

## About the Project

<p align="center">
  <a href="https://github.com/robotcraft19/robotcraft-pathfinding>
    <img src="res/images/..." alt="About" width="300" height="160">
  </a>
</p>

This package was developed during the Robotcraft '19 program. Using a P(ID) wall follower navigation method first, the robot maps the environment and avoid collisions. Then, using the map, an A* search is performed and an optimal path is found, solving a maze effectively.

## Setup

Install ROS Melodic, you can also use tutorials. There's a bunch of them, including in the ros wiki. I may add a script later in a repository.

Now that you have ROS, to setup the project on your local machine:

1. Click on `Fork`.
2. Go to your fork and `clone` the project to your local machine, in the "catkin_ws" folder.
3. `git clone https://github.com/robotcraft19/robotcraft-pathfinding.git`
4. Make sure you have rosdep install : `sudo apt-get install python-rosdep && sudo rosdep init`
5. `cd ~/catkin_ws`
6. `rosdep install --from-paths src --ignore-src -r -y`
7. In the catkin workspace : `catkin_make`

If everything went smoothly, you should now have this repo's package as well as its dependencies.

## Run

Running the nodes is quite easy as launch files were made. 

### First Step :

- Reactive Driving and Mapping : `roslaunch robotcraft-pathfinding maze_basic.launch`

A node then stores the mapped environment in the scans folder. The initial position used by the pathfinder is the initial position of the robot. The final position is the last known position. It is saved when exiting the task and repetitively throughout the process.

### Second Step :

Another node loads the mapped environment stored previously at the begining of the astar pathfinding launch file.

- A* Pathfinder : `roslaunch robotcraft-pathfinding maze_pro.launch`

## Roadmap

See the [open issues](https://github.com/robotcraft19/robotcraft-pathfinding/issues) for a list of proposed features (and known issues).

## Contribute

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

### Contribute on proposed features

1. Choose any open issue from [here](https://github.com/robotcraft19/robotcraft-pathfinding/issues). 
2. Comment on the issue: `Can I work on this?` and get assigned.
3. Make changes to your fork and send a PR.

Otherwise just create the issue yourself, and we'll discuss and assign you to it if serves the project !

To create a PR:

Follow the given link to make a successful and valid PR: https://help.github.com/articles/creating-a-pull-request/

To send a PR, follow these rules carefully, **otherwise your PR will be closed**:

1. Make PR title in this formats: 
```
Fixes #IssueNo : Name of Issue
``` 
```
Feature #IssueNo : Name of Issue
```
```
Enhancement #IssueNo : Name of Issue
```

According to what type of issue you believe it is.

For any doubts related to the issues, i.e., to understand the issue better etc, comment down your queries on the respective issue.

## License

Distributed under the MIT License. See `LICENSE` for more information.

## Contact

Erwin Lejeune - [@spida_rwin](https://twitter.com/spida_rwin) - erwin.lejeune15@gmail.com

## Contributors

Everyone part of the original team or that assisted throughout the development.

- [Nicolas Filliol (Project Lead)](https://github.com/nicofilliol)
- [Erwin Lejeune](https://github.com/Guilyx)

[contributors-shield]: https://img.shields.io/github/contributors/robotcraft19/robotcraft-pathfinding.svg?style=flat-square
[contributors-url]: https://github.com/robotcraft19/robotcraft-pathfinding/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/robotcraft19/robotcraft-pathfinding.svg?style=flat-square
[forks-url]: https://github.com/robotcraft19/robotcraft-pathfinding/network/members
[stars-shield]: https://img.shields.io/github/stars/robotcraft19/robotcraft-pathfinding.svg?style=flat-square
[stars-url]: https://github.com/robotcraft19/robotcraft-pathfinding/stargazers
[issues-shield]: https://img.shields.io/github/issues/robotcraft19/robotcraft-pathfinding.svg?style=flat-square
[issues-url]: https://github.com/robotcraft19/robotcraft-pathfinding/issues
[license-shield]: https://img.shields.io/github/license/robotcraft19/robotcraft-pathfinding.svg?style=flat-square
[license-url]: https://github.com/robotcraft19/robotcraft-pathfinding/blob/master/LICENSE.md
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=flat-square&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/erwinlejeune-lkn
