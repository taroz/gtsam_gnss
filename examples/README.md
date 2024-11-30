# Requirements
- [MatRTKLIB](https://github.com/taroz/MatRTKLIB):
MATLAB wrapper for RTKLIB.
```shell
git clone https://github.com/taroz/MatRTKLIB.git
```
Add the MatRTKLIB installation directory to the MATLAB search path.
- [MatlabProgressBar](https://github.com/JAAdrian/MatlabProgressBar):
tqdm like MATLAB progress bar.
```shell
git clone https://github.com/JAAdrian/MatlabProgressBar.git
```
Add the MatlabProgressBar installation directory to the MATLAB search path.

- In the case of Linux, due to the linker issue shown [here](https://github.com/borglab/gtsam/blob/develop/matlab/README.md), you need to run the following shell line before starting MATLAB from the same shell.
```shell
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6
matlab
```

# Examples
- `estimate_pos.m`: Estimate position and receiver clock using `PseudorangeFactor_XC`