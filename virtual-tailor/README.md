## Welcome to the Virtual Tailor Project!

The <a href="https://www.researchgate.net/publication/279201882_Tailleur_Virtuel_Modelisation_3D_d'un_corps_humain" target="_blank">Virtual Tailor Project</a> is a tool for measuring human body.

### This is a description of the files/directories:

* modeling: the implementation of [DAO's
  method](http://ieeexplore.ieee.org/xpl/articleDetails.jsp?tp=&arnumber=7041571&url=http%3A%2F%2Fieeexplore.ieee.org%2Fxpls%2Fabs_all.jsp%3Farnumber%3D7041571)

* README.md: this file.

* take-coor: for taking coordinates.

* measure: for measuring.

### How to compile:

In each ``src`` directory 

    $ cmake -DCMAKE_BUILD_TYPE=Release ../
    $ sudo make

### How to run:

* For ``modeling`` :

Just place 4 clouds of human body and

    $ ./main

* For ``take-coor`` :

``file.pcd`` is merged cloud. (Don't forget place file.pcd when taking coordinates)

    $ ./pcl_visualizer_demo -i

* For ``measure`` :

Need output of ``take-coor ``!
An example of usage:

    $ ./main -0.124821 0.0418782 0.219404 0.0347325 -0.118073 -0.124766 0.204303 -0.127636 -0.15441 -0.363924 0.24741 -0.360937
