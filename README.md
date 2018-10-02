# autodifk
A lightweight C++ tool for automatic differentiation.

[![Build Status](https://travis-ci.org/dfridovi/mininet.svg?branch=master)](https://travis-ci.org/dfridovi/autodifk)
[![License](https://img.shields.io/badge/license-BSD-blue.svg)](https://github.com/dfridovi/autodifk/blob/master/LICENSE)

## Status
**autodifk** is still under active development.

## Dependencies
I may miss a few here, but here is a list of dependencies:

* [Eigen](http://eigen.tuxfamily.org/dox/) (header-only linear algebra library)
* Gflags (Google's command-line flag manager)
* Glog (Google's logging tool)

## Usage
You'll need to begin by building the repository. From the top directory, type the following sequence of commands:

```
mkdir bin
mkdir build
cd build
cmake ..
make -j4
```

This should build all tests and executables. In order to run tests, you can run the following command:

```
./run_tests
```

from within the `build/` directory you just made. All the tests should pass, and none should take more than a second or so to run.

## API documentation
I'm using Doxygen to auto-generate web-based [documentation](https://dfridovi.github.io/autodifk/documentation/html/).
