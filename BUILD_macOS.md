# Build GTSAM and gtsam_gnss on macOS

## Test environments
- macOS (Sequoia 15.2)
  - MacBook (Apple M4 PRO)
- MATLAB 2024b

## Requirement
- Xcode 16.2

## Installing Required Packages
```shell
brew install boost tbb cmake
pip3 install pyparsing
```

## Build GTSAM
```shell
git clone https://github.com/taroz/gtsam-4.3a.git
cd gtsam-4.3a
mkdir build_macOS && cd build_macOS
cmake .. -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_TESTS=OFF -DGTSAM_INSTALL_MATLAB_TOOLBOX=ON
make -j
sudo make install
```
`gtsam_toolbox` is installed in `/usr/local/`

## Build gtsam-gnss
```shell
git clone https://github.com/taroz/gtsam_gnss.git
cd gtsam_gnss
mkdir build_macOS && cd build_macOS
cmake ..
make
sudo make install
```
By default, `gtsam_gnss` is installed in `user/local/gtsam_toolbox`.
Add `user/local/gtsam_toolbox` to your MATLAB search path.
