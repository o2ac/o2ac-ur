# GPU Voxels installation
### Dependencies
1. Eigen3: Download from [here](http://eigen.tuxfamily.org/index.php?title=Main_Page#Download) (eigen-3.4-rc1)

```
mkdir build
cd build
cmake ..
make install
```

2. GLM `apt-get install libglm-dev`

### Install from source
    export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/root/o2ac-ur/eigen-3.4-rc1/build
    git clone https://github.com/fzi-forschungszentrum-informatik/gpu-voxels.git
    cd gpu-voxels
    
    mkdir build
    cd build
    
    cmake ..
    make
