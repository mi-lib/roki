RoKi - Robot Kinetics library
Copyright (C) 1998 Tomomichi Sugihara (Zhidao)

-----------------------------------------------------------------
[What is this?]

RoKi is a software library for robot kinetics computation including:

 - forward kinematics
 - velocity and acceleration analysis
 - inverse dynamics
 - inverse kinematics
 - forward dynamics
 - collision detection and contact dynamics

It facilitates on-line robot controls as well as off-line robot
motion analyses, plannings and simulations.

ZEDA, ZM and Zeo are required.

-----------------------------------------------------------------
[Installation / Uninstallation]

<install>
0. Install ZEDA, ZM and Zeo in advance.

1. Unpack the distributed archive where you want.

% zcat roki-X.Y.Z.tgz | tar xvf
or, if you use GNU tar,
% tar xzvf roki-X.Y.Z.tgz

X.Y.Z is for the revision.

2. Enter the directory.

% cd roki-X.Y.Z

3. Edit config file if necessary.
  PREFIX   directory where the library is installed.
           ~/usr as a default. In this case, header files
           and library are installed under ~/usr/include
           and ~/usr/lib, respectively.

4. Make it.

% make

5. Install it.

% make install

Or,

% cp -a lib/libroki.so $PREFIX/lib/
% cp -a include/roki $PREFIX/include/
% cp -a bin/* $PREFIX/bin/

<uninstall>
Delete $PREFIX/lib/libroki.so and $PREFIX/include/roki.

-----------------------------------------------------------------
[How to use]

You may need to set your PATH and LD_LIBRARY_PATH environment
variables. This is done by:
 export PATH=$PATH:$PREFIX/bin
 export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$PREFIX/lib
if your working shell is Bourne shell (bash, zsh, etc.), or by:
 set path = ( $path $PREFIX/bin )
 setenv LD_LIBRARY_PATH $LD_LIBRARY_PATH:$PREFIX/lib
if your working shell is C shell (csh, tcsh, etc.).

When you want to compile your code test.c, for example, the following
line will work.

% gcc `roki-config -L` `roki-config -I` test.c `roki-config -l`

-----------------------------------------------------------------
[Contact]

zhidao@ieee.org
