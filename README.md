# Berkeley Path Planning
[![License](https://img.shields.io/badge/license-BSD-blue.svg)](LICENSE)

Berkeley Path Planning is a robotics path planning library dedicated to exploring new solutions to path optimization. The target application is to navigate a quadrotor based on video, GPS, and IMU, optimizing the path such that the quadrotor takes pictures for an optimal 3D reconstruction. It is developed by **David Fridovich-Keil**, a first year graduate student in the [Video and Image Processing Lab](http://www-video.eecs.berkeley.edu) at UC Berkeley.

## Overview
This repository is entirely written in C++, and is structured around the CMAKE cross-compilation paradigm. All source code is in the `path_planning/src/cpp/` directory. We have also written extensive unit tests, which are automatically compiled and tested in continuous integration. The source code for these tests is stored in the `path_planning/test/` directory, and the test executable will be compiled into `path_planning/build/run_tests`. Tests can be run using `make check`.

## Status
This library is currently in development, and right now it really doesn't do all that much. Don't worry! I'm working on it...


## Build Instructions
We follow the standard CMAKE build scheme. Just download the repository and from the top directory type:

```bash
mkdir build && cd build && cmake .. && make
```

Optionally, to run tests:

```bash
make check
```
