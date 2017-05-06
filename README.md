# fotokite
Stabilize Fotokite in respect to AprilTag or automatically detected features. Contains Fotokite class, which is the C++ interface to Fotokite. It can be used to get current state of Fotokite and to send commands to Fotokite. Two communication methods are implemented. The first is serial communication over USB serial port. The second is socket communication using OCU Server over Ethernet.

## OpenCV Installation on macOS

1. Install Homebrew:

    /usr/bin/ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"

2. Install OpenCV:

    brew tap homebrew/science

    brew install opencv3 --with-contrib --with-ffmpeg --with-tbb --with-qt5

3. Link OpenCV:

    brew link --force --override opencv3

4. Install CMake:

    https://cmake.org

5. In Terminal, change directory into the root directory of the project and run the following command to generate makefile:

    cmake .

6. Compile the project:

    make

7. Run:

    ./Fotokite

## Open in NetBeans

File

Open Project

Go to the source directory

Open Project
