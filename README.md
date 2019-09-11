# Capstone Project: Elastic Collisions of Multiple Balls Simulation

This project is the capstone project as part of the **C++ Developer Nanodegree**, by **Udacity**. It allows users to simulate elastic collisions of multiple balls, with up to one hundred balls, which roughly simulate a gas confined in a cylinder, The Kinetic Theory of Gases and The First Law of Thermodynamics.

The program has two simulation modes. In the standard, the program uses these equations to the movement of balls and the piston:

    m0*v0i + m1*v1i = m0*v0f + m1*v1f   (Conservation of Linear Momentum)
    Ec0i + Ec1i = Ec0f + Ec1f           (Conservation of Kinetic Energy)

In the "gas" mode, the piston moves according to the force exerted on it by the balls. The program uses this equation for the piston:

    F*(tf-ti) = m0*(v0i-v0f)   m0->ball  (Newton's Second Law; Impulse Theorem)
    F = m1*a                   m1->piston

This is one possible output:

<center><img src="multiple_balls_collision.gif" alt="Cylinder with gas simulation" width="300"/></center>

<br />

It is possible to approximately simulate an elevation of the system energy, by applying some "heat" at the bottom (use the left-key). The bottom color will change to light red, and the minimum ball velocity will increase by some value.

# Installation

## Dependencies for Running Locally

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac)
  * Linux: make is installed by default on most Linux distros
  * Mac: [according to references, install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
* SDL2 >= 2.0
  * All installation instructions can be found [here](https://wiki.libsdl.org/Installation)
  * Note that for Linux, an `apt` or `apt-get` installation is preferred to building from source.
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)

* Windows: we have used the **VisualStudio IDE** (https://visualstudio.microsoft.com/), which uses the MSVC compiler, its configuration files or also the `Cmake`, and can target a Linux machine for compilation over network. But it needs some configuration to locate the SDL2 library. Instructions below.

* The final files three is like this:

      <ROOT PROJECT FOLDER>
          <src>
              Ball.cpp
              Ball.h
              Constants.h
              CylinderObject.cpp
              CylinderObject.h
              Globals.h
              Main.cpp
              Parameters.txt
              Wall.cpp
              Wall.h  
          <build>
              Parameters.txt
              (compiler output files)
          CmakeLists.txt
          README.md

## Basic Build Instructions

1. Copy all the files and `src` folder in a new folder.
2. Make a `build` directory in the top level directory: `mkdir build`.
3. Change to the `build`: `cd build`
4. Generate the make files: `cmake ..`
5. Compile: `make`
6. Copy the file `Parameters.txt` from `src` to the `build` folder
7. Run it: `./CapstoneProjectCmake`.

## Installation on Windows and VisualStudio, to create a Cmake project:

On **VisualStudio**, create a folder and save the project files. Open the folder with the Visual Studio.
The **VisualStudio** will see the 'Cmake' and will try to configure itself, but will show an error.
In the configuration GUI of the 'Cmake' (Project->Cmake settings), see the SDL2_DIR variable. Point that
to the directory where you extracted the SDL2 development package. In that folder, also create a file `sdl2-config.cmake` and write:

    set(SDL2_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/include")

    # Support both 32 and 64 bit builds
    if (${CMAKE_SIZEOF_VOID_P} MATCHES 8)
      set(SDL2_LIBRARIES "${CMAKE_CURRENT_LIST_DIR}/lib/x64/SDL2.lib;${CMAKE_CURRENT_LIST_DIR}/lib/x64/SDL2main.lib")
    else ()
      set(SDL2_LIBRARIES "${CMAKE_CURRENT_LIST_DIR}/lib/x86/SDL2.lib;${CMAKE_CURRENT_LIST_DIR}/lib/x86/SDL2main.lib")
    endif ()

    string(STRIP "${SDL2_LIBRARIES}" SDL2_LIBRARIES)

 This will configure `Cmake` (after have went there by the SDL2_DIR) to point to the SDL libraries. Now the error was gone.
 See detailed instructions [here](https://trenki2.github.io/blog/2017/06/02/using-sdl2-with-cmake/)

If we want to use the standard `vcxproj` buildings, we can install the `SDL2` with the `vcpkg` (https://github.com/Microsoft/vcpkg) and `vcpkg integrate install`, to automatically configure the IDE. But we have to manually link the `SDL2maind.lib` (for debug) or `SDL2main.lib` for release configurations (Property->Linker->Input->Additional Dependencies->'SDL2maind.lib'). Also, we have to change the `#include <SDL.h>` to `#include <SDL2\SDL.h>`. These are the only additional adjust.

# Common usage

The `Parameters.txt` file, that should be located at the same folder as the executable, will contain parameters that the user can adjust,
like the number of balls, the initial ball speed, the piston or the ball mass, and others.

There is a `menu` with some example simulations, but it is possible to load parameters from a file.

      Please, choose one example or read parameters from file:
      [1] Three balls. (default)
      [2] One ball at rest. Piston with gravity.
      [3] Eight balls at rest. Piston with gravity.
      [4] Fifty balls (gas simulation approximation). Piston with gravity.
      [5] Load parameters from file.

      Option:

If the user chooses only two balls, and set the IS_TESTING to 1, the user can adjust the speed and angle of the velocity vector of the two balls,
to simulate a specific collision. If the IS_TESTING is 0, the velocities will be randomly configured.

After choosing the option, the program will print the parameters for the user to verify. This is an example:

      Please, verify the simulation parameters:

      GAS_MODE 0
      NBALLS 3
      BALL_RADIUS 20
      BALL_SPEED 60
      BALL_MASS 0.1
      BALL_GRAVITY 0
      IS_TESTING 0
      BALL0_SPEED 10
      BALL0_VEL_ANGLE 0
      BALL1_SPEED 10
      BALL1_VEL_ANGLE -180
      PISTON_MASS 100
      PISTON_GRAVITY 0
      BOTTOM_TEMP_MIN 0
      BOTTOM_TEMP_MAX 100

      Press enter to continue...


As physical simulation, the complexity is great and the software is slightly inexact, because we can see a very small decrease in energy of the system.
We can put energy, as approximation, increasing the cylinder bottom temperature, with the left key.

During simulation, extra data about the mean force applied by the balls is printed to the console:


    Time:0  balls:80        gas avg(1.0s) up force : 37.76          piston down force: 100.00       ~temp(bottom) : 0.00
    Time:1  balls:80        gas avg(1.0s) up force : 76.65          piston down force: 100.00       ~temp(bottom) : 0.00
    Time:2  balls:80        gas avg(1.0s) up force : 85.07          piston down force: 100.00       ~temp(bottom) : 0.00
    Time:3  balls:80        gas avg(1.0s) up force : 88.65          piston down force: 100.00       ~temp(bottom) : 0.00
    Time:4  balls:80        gas avg(1.0s) up force : 76.72          piston down force: 100.00       ~temp(bottom) : 0.00
    Time:5  balls:80        gas avg(1.0s) up force : 108.61         piston down force: 100.00       ~temp(bottom) : 0.00
    Time:6  balls:80        gas avg(1.0s) up force : 149.78         piston down force: 100.00       ~temp(bottom) : 0.00
    Time:7  balls:80        gas avg(1.0s) up force : 75.26          piston down force: 100.00       ~temp(bottom) : 0.00
    Time:8  balls:80        gas avg(1.0s) up force : 134.64         piston down force: 100.00       ~temp(bottom) : 0.00
    Time:9  balls:80        gas avg(1.0s) up force : 120.86         piston down force: 100.00       ~temp(bottom) : 0.00
    Time:10 balls:80        gas avg(1.0s) up force : 70.12          piston down force: 100.00       ~temp(bottom) : 0.00

Each ball will run in a separate thread, so the program uses concurrent computing. 🚩

# Some points addressed by the program

* Compiling and Testing
  * The project compiles without errors, possibly on any platform, because it uses the `cmake` and `make`. We have just compiled in **Ubuntu** for testing.
* Loops, Functions, I/O
  * We have `resolveCollision()`, `circleCircleCollision()`, and `squareCircleCollision()` as functions to help the calculation of velocities and positions, as an example of code organization, in the `Ball.cpp`. We have `chooseSimulations()`, `createObject()`, `processPiston()` as example in the `Main.cpp`, about others.
  * The program read data from an external file, in the function `loadParametersFromFile()`, line 589 in `Main.cpp`. The program has a `Menu` with example of simulations, reading data from user, in `chooseSimulations()`, line 689 of `Main.cpp`.
  * We have a infinite `while-loop` in `main()`, line 99 and 105, which exits when we close the simulation window. It also has The same kind of loop is present at the `Ball::play()`, line 142 of `Ball.cpp`, and `processPiston()`, line 436 of `Main.cpp`. These loops have a stop watch and have a cycle of a definite time, when the computations are processed.
* Objected Oriented Programming
    * The Balls, the cylinder walls, the piston and the bottom of the cylinder are all objects of some class. Each ball moves itself based on its function `Ball::play()`. Nevertheless, each ball runs on a separate thread, and also the piston, which is a `Wall` object, and runs in the `processPiston()` function, line 436 of `Main.cpp`.
    * All classes are declared in header `.h` files and implemented in source `.cpp` files, organizing the code.
    * All class data members are specified as public, protected or private, as we can see in `cylinderObject.h`, `Wall.h` and `Ball.h`. The member functions are documented through function names and comments, as example the `Ball` class (declared in `Ball.h`).
    * The cylinder objects were group as their characteristics. The balls are in movement and have spherical shape. They are in the `Balls` class.
    The walls, are rectangular. They are in `Wall` class. Both are derived form `CylinderObject` class, which has a common characteristic, it has a position, a mass, and can have a speed, as in case of the piston.
    * In the `Ball.h` file, we have the `MessageQueue` class (lines 15-27),  declared with a template that allows it to accept a generic parameter.
* Memory Management
    * In the `createObject()` function, at line 202 of `Main.cpp`, all four parameters (vectors) are passed by reference. This is necessary because the function will create new vector members, updating the content of the parameters.
    * The program uses `std::shared_ptr` to reference the balls, walls, piston and bottom, the objects of simulation. As an example, the lines 57-60 of `main()`, in the `Main.cpp` file.
    * The program does not use raw pointers.
* Concurrency
    * Each ball, object of `Ball` class, runs on a separate thread, as can be viewed starting at line 81 of `Main.cpp`, which calls `Ball::simulate` that calls a thread that runs `Ball::play`, thread at line 121 of `Ball.cpp`. The threads are stored in a `_threads` vector, which belongs to the `CylinderObject` class. The destructor of the `CylinderObject` set up a thread barrier before the object is destroyed, calling the `join()` for each thread, line 71-72 of the `CylinderObject.cpp`.
    * The class `MessageQueue`, declared in the `Ball.h` and implemented at `Ball.cpp`, uses as member variables a `std::mutex` and a `std::condition_variable`. In the `receive()` method, it uses a `std::unique_lock` to manage  the `mutex` and temporally unlock it when inside the `wait()` method of the `condition_variable`. This is necessary for the `queue` be updated. When the `queue` is not empty, so we have new data, the `unique_lock` resume the lock to get the message from `queue`. The `queue` is used for communications with the `processPiston()`, line 499-514 of `Main.cpp`, through the `Ball::dataIsAvailable` and `Ball::receiveMsg` member functions. It will inform the piston about the ball collisions. The piston will get the information of the velocity, and process according to each cycle, time computed, to calculate the force that the balls are pushing on it, in the `gas` mode simulation.
    * In the same `processPiston()`, another thread is launched with a Lambda function, to print information about the simulation, line 456-487.

That's it! 🎥

**This app is for learning purposes.** 📚


# Credits

Halliday, Resnick and Walker. Physics for JEE (Main & Advanced). Adapted by Amit Gupta. Vol. I. Ch. 9, p. 310. Ch. 20, p. 776. Ch 21, p. 796, fig. 21-1.

This is an excellent explanation about elastic collisions:

https://blogs.msdn.microsoft.com/faber/2013/01/09/elastic-collisions-of-balls/

These are some useful links, in addition to **Udacity** itself, that were queried in this project:

https://lazyfoo.net/tutorials/SDL/01_hello_SDL/linux/index.php

https://stackoverflow.com/questions/38334081/howto-draw-circles-arcs-and-vector-graphics-in-sdl

https://www.libsdl.org/tmp/SDL/VisualC.html

https://www.libsdl.org/release/SDL-1.2.15/docs/html/guideinputkeyboard.html

https://github.com/udacity/CppND_Capstone_Snake_Game

http://www.cplusplus.com/doc/tutorial

https://stackoverflow.com/questions/7412548/error-gnu-stubs-32-h-no-such-file-or-directory-while-compiling-nachos-source

https://trenki2.github.io/blog/2017/06/02/using-sdl2-with-cmake/

https://github.com/Zottel/sdl2_minimal/blob/master/example.cpp