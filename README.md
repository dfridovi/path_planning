# Berkeley Path Planning
[![License](https://img.shields.io/badge/license-BSD-blue.svg)](LICENSE)

Berkeley Path Planning is a robotics path planning library dedicated to exploring new solutions to path optimization. The target application is to navigate a quadrotor based on video, GPS, and IMU, optimizing the path such that the quadrotor takes pictures for an optimal 3D reconstruction. It is developed by **David Fridovich-Keil**, a first year graduate student in the [Video and Image Processing Lab](http://www-video.eecs.berkeley.edu) at UC Berkeley, and **James Smith**, a third year undergraduate at UC Berkeley.

## Overview
This repository is entirely written in C++, and is structured around the CMAKE cross-compilation paradigm. All source code is in the `path_planning/src/cpp/` directory. We have also written extensive unit tests, which are automatically compiled and tested in continuous integration. The source code for these tests is stored in the `path_planning/test/` directory, and the test executable will be compiled into `path_planning/build/run_tests`. Tests can be run using `make check`.

## Status
This library is currently in development. We have recently ported all code to work with Point Cloud Library (PCL). Currently, the library supports 2D planning in continuous environments where obstacles are modeled as Gaussians.

Future work will be devoted to mapping via merging odometry estimates from point cloud alignment and from GPS and other sources.

## Build Instructions
We follow the standard CMAKE build scheme. Just download the repository and from the top directory type:

```bash
mkdir build && cd build && cmake .. && make
```

Optionally, to run tests:

```bash
make check
```

## Acknowledgements
We have borrowed some of the code from the `berkeley_sfm` project, ongoing work by **Erik Nelson** and **David Fridovich-Keil**.
