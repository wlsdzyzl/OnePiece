## OnePiece

One-Piece Library. 

RGB-D based 3D vision library, you can easily use OnePiece to build a SLAM system and extract a 3D model.

![](./fba_fusion.gif)

Click [documentation(Chinese)](http://wlsdzyzl.top/onepiece.github.io/) or [documentation(English)](http://wlsdzyzl.top/onepiece.github.io/en/) for more information.

Watch some demos [here](http://wlsdzyzl.top/onepiece.github.io/examples).

You can download `TestData` [here](https://drive.google.com/file/d/1eGKY3IEp4PYlxEI-Os0cP2s6gpi-tXmP/view?usp=sharing). 



### Get started
Requirements:
- OpenCV 3.4
- OpenGL
- OpenNI2 (optional, if you want to use a asus xtion camera to capture real data)
- Other small 3rd libraries are packaged into `3rdparty`
```
mkdir build
cd build
cmake .. && make -j
```
#### NOTE:
You should know that, anaconda may install its own libpng, this can cause conflictions with the original libpng, which is used by 3rd libraries. Using following command and rebuilding the whole project may be helpful:
```
conda uninstall libpng
```
### License
The code is free to everyone, however, I have used MILD which could have some limitations on commercial usages. See [license](https://github.com/lhanaf/MILD/blob/master/LICENSE.txt).
