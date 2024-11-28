# Build GTSAM and gtsam_gnss on Windows
Building GTSAM on Windows is a bit more complicated than on linux. In the environments I have tried, the computation speed is slower on Windows than on linux. Also, depending on the build environment, the mex file may output unknown errors. For this reason, I cannot guarantee that gtsam_gnss will work on Windows.

## Test environments
- Windows 11, 64bit
- MATLAB 2024a

## Requirement
- [Visual Studio 2022 Community Edition](https://visualstudio.microsoft.com/ja/vs/community/)
- [Python 3.XX](https://www.python.org/downloads/windows/)

## Install pyparsing
```shell
pip install pyparsing
```

## Install Boost
1. Download Boost pre-built binaries from [here](https://sourceforge.net/projects/boost/files/boost-binaries/)
    - Version: 1.86.0, MSVC 14.3 (VS2022), 64 bit
      - [`boost_1_86_0-msvc-14.3-64.exe`](https://sourceforge.net/projects/boost/files/boost-binaries/1.86.0/boost_1_86_0-msvc-14.3-64.exe)
2. Install Boost to any directory
3. Add the following path to the Windows environment variable
`Boost_DIR` : `path-to-install\boost_1_86_0`

## Build GTSAM
1. Clone or download [gtsam-4.3a](https://github.com/taroz/gtsam-4.3a)
    - Due to the existence of build problems with the MATLAB wrapper, please use GTSAM from the above repository instead of the original GTSAM

2. Launch `x64 Native Tools Command Prompt for VS 2022` from the Windows start menu with **administrative permissions**.
```shell
cd path-to-install\gtsam-4.3a
cmake . -B build_windows -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_TESTS=OFF -DGTSAM_INSTALL_MATLAB_TOOLBOX=ON -DGTSAM_WITH_TBB=OFF
cmake --build build_windows --config Release
cmake --install build_windows
```
3. GTSAM and gtsam_toolbox is installed in `C:\Program Files (x86)\GTSAM`. Copy the DLL to the mex file (gtsam_wrapper.mexw64) directory.
```shell
copy "C:\Program Files (x86)\GTSAM\bin\*.dll" "C:\Program Files (x86)\GTSAM\gtsam_toolbox"
```

## Build gtsam_gnss
1. Clone or download [gtsam_gnss](https://github.com/taroz/gtsam_gnss)
2. Launch `x64 Native Tools Command Prompt for VS 2022` from the Windows start menu with **administrative permissions**.
```shell
cd path-to-install\gtsam-gnss
cmake . -B build_windows
cmake --build build_windows --config Release
cmake --install build_windows
```
3. gtsam_gnss is installed in `C:\Program Files (x86)\GTSAM\gtsam_toolbox`.
4. Add the above path to your MATLAB search path.