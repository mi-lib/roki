RoKi - Robot Kinetics library
=================================================================
Copyright (C) Tomomichi Sugihara (Zhidao) since 1998

-----------------------------------------------------------------
## [What is this?]

RoKi is a software library for robot kinetics computation including:

- forward kinematics
- velocity and acceleration analysis
- inverse dynamics
- inverse kinematics
- forward dynamics
- collision detection and contact dynamics

It facilitates on-line robot controls as well as off-line robot
motion analyses, plannings and simulations.

ZEDA, ZM and Zeo are required to be installed.

-----------------------------------------------------------------
## [Installation / Uninstallation]

### install

Install ZEDA, ZM and Zeo in advance.

Move to a directly under which you want to install RoKi, and run:

   ```
   % git clone https://github.com/zhidao/roki.git
   % cd roki
   ```

Edit **PREFIX** in *config* file if necessary in order to specify
a directory where the header files, the library and some utilities
are installed. (default: ~/usr)

   - header files: $PREFIX/include/roki
   - library file: $PREFIX/lib
   - utilities: $PREFIX/bin

Then, make and install.

   ```
   % make && make install
   ```

### uninstall

Do:

   ```
   % make uninstall
   ```

which removes $PREFIX/lib/libroki.so and $PREFIX/include/roki.

-----------------------------------------------------------------
## [How to use]

When you want to compile your code *test.c*, for example, the following line will work.

   ```
   % gcc `roki-config -L` `roki-config -I` test.c `roki-config -l`
   ```

-----------------------------------------------------------------
## [Contact]

zhidao@ieee.org
