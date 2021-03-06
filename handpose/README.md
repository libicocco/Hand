Hand Pose Estimation and Rendering
==========================================

* Author:    Javier Romero (<jrgn@kth.se>)
* GitHub:    <https://github.com/libicocco/Hand>

This free software is copyleft licensed under version 3 of the GPL license.

The goal of the software is to estimate the pose (as joint angles) of a human
hand present in an image. No markers are needed. The output can be given as 
joint angle or a rendered image with the estimated joint angles.

It requires [buola][1] by Xavi Gratal to be built. Apart from these, it depends
on gsl, eigen3 and a c++0x capable compiler. 

[1]: https://github.com/gratal/buola


Usage:
------

You should first download this source into some directory
    
    export HAND_PATH=$HOME
    mkdir -p $HAND_PATH
    cd $HAND_PATH
    git clone https://libicocco@github.com/libicocco/Hand.git
    
Then build it:

    mkdir $HAND_PATH/Hand/handpose/{build,out}
    cd $HAND_PATH/Hand/handpose/build
    cmake ..
    make -j5

You can build the basic rendering database:

    cd $HAND_PATH/Hand/handpose
    build/apps/createRenderDB

And use it to create the big database:

    rm -r /tmp/out/ ; build/scene/renderhand  --outpath /tmp/out/ 

Contributors:
-------------

* Javier Romero [2]

[2]: https://github.com/libicocco


Discussion:
-----------

Lots of things to do still!


TODO:
-----

1. Improve the usability.

2. Improve robustness.

3. Integrate the rendering engine to provide online corrections to the discriminative approach.
