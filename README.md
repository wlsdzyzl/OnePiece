## OnePiece

One-Piece Library. 

RGB-D based 3D vision library, you can easily use OnePiece to build a SLAM system and extract a 3D model.

![](./fba_fusion.gif)

Click [documentation(Chinese)](http://wlsdzyzl.top/onepiece.github.io/) or [documentation(English)](http://wlsdzyzl.top/onepiece.github.io/en/) for more information.

Watch some demos [here](http://wlsdzyzl.top/onepiece.github.io/examples).

You can download `TestData` [here](https://cloud.tsinghua.edu.cn/f/a2372da684f14330af21/?dl=1). 



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
Under the GPL License, see [http://opensource.org/licenses/GPL](http://opensource.org/licenses/GPL).
