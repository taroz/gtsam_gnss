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
- Before running MATLAB script, please check that the following two paths have been added to the MATLAB search path.
```shell
/path/to/MatRTKLIB
/path/to/MatlabProgressBar
```
# Examples
- `estimate_pos_P.m`: Estimate position (x) and receiver clock (c) using `PseudorangeFactor_XC` and `ClockFactor_CC`
- `estimate_pos_PD.m`: Estimate position (x) and receiver clock (c) using `PseudorangeFactor_XC`, `ClockFactor_CC`, and `DopplerFactor_XXCC`
- `estimate_pos_PDC.m`: Estimate position (x) and receiver clock (c) using `PseudorangeFactor_XC`, `ClockFactor_CC`, `DopplerFactor_XXCC`, and `TDCPFactor_XXCC`
- `estimate_pos-vel_PD.m`: Estimate position (x), velocity (v), receiver clock (c), and receiver clock drift (d) using `PseudorangeFactor_XC`, `DopplerFactor_VD`, `MotionFactor_XXVV`, and `ClockFactor_CCDD`
- `estimate_pos-vel_PDC.m`: Estimate position (x), velocity (v), receiver clock (c), and receiver clock drift (d) using `PseudorangeFactor_XC`, `DopplerFactor_VD`, `MotionFactor_XXVV`, `ClockFactor_CCDD`, and `TDCPFactor_XXCC`
- `estimate_vel_D.m`: Estimate velocity (v) and receiver clock drift (d) using `DopplerFactor_VD`
- `estimate_pos_ambiguity_PC.m`: Estimate position (x) and ambiguity (b) using `PseudorangeFactor_X` and `CarrierPhaseFactor_XB`
- `estimate_pos_vel_ambiguity_PDC.m`: Estimate position (x), velocity (v), and ambiguity (b) using
`PseudorangeFactor_X`, `DopplerFactor_V`, `CarrierPhaseFactor_XB`, `TDCPFactor_XX`, and `MotionFactor`
- `paper_example1_robust_error_model.m`: Example 1 in [PLANS paper](https://arxiv.org/pdf/2502.08158): Comparison with and without robust error model
- `paper_example2_ambiguity_resolution.m`: Example 2 in [PLANS paper](https://arxiv.org/pdf/2502.08158): Comparison of ambiguity resolution for different models