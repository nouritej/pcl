About
-----------------------------------
Example shows how to capture a point cloud and visualize it.

You will need a common Kinect device,PCL and OpenNI.

For this code, you'r going to use PCL with the OpenNI drivers.


Compiling
-----------------------------------

```sh
cd src
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

Usage
-----------------------------------

To take a cloud data with 2-second timer and visualize it:
```sh
./openniViewer -t 2
./openniViewer -v inputCloud0.pcd
```
