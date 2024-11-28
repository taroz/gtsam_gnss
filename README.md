# gtsam_gnss
This repository is a small set of custom factors and MATLAB wrappers that use [GTSAM](https://github.com/borglab/gtsam) for GNSS processing. 

This repository is currently under development, so we will gradually add content.

# Test environments
- Ubuntu 22.04
- MATLAB 2024a

# Build
## GTSAM
- [GTSAM](https://github.com/borglab/gtsam):
Factor graph optimization library. Due to a problem with the MATLAB wrapper, please clone (GTSAM from my repository)[https://github.com/taroz/gtsam-4.3a] instead of the original GTSAM and build it using the following procedure.
```shell
sudo apt-get install -y git build-essential cmake libboost-all-dev libtbb-dev
pip install pyparsing
git clone https://github.com/taroz/gtsam-4.3a.git
cd gtsam-4.3a
mkdir build && cd build
cmake .. -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_TESTS=OFF -DGTSAM_INSTALL_MATLAB_TOOLBOX=ON
make -j$(nproc)
sudo make install
```
`gtsam_toolbox` is installed in `/usr/local/`

## gtsam-gnss
```shell
git clone https://github.com/taroz/gtsam_gnss.git
cd gtsam_gnss
mkdir build && cd build
cmake ..
make
sudo make install
```
By default, `gtsam_gnss` is installed in `user/local/gtsam_toolbox`.
Add `user/local/gtsam_toolbox` to your MATLAB search path.

# Build on Windows
Building GTSAM and gtsam_gnss on Windows is a little complicated. The procedure is shown [here](./BUILD_WINDOWS.md).

# Examples
- See [examples](./examples) directory
- Use in the Google Smartphone Decimeter Challenge. See [gsdc2023](../gsdc2023) repository

# Citation
T. Suzuki, "Open-Source Factor Graph Optimization Package for GNSS: Examples and Applications," 2025 IEEE/ION Position, Location and Navigation Symposium (PLANS), (accepted)