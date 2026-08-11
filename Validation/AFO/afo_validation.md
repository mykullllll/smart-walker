# Feedforward Validation Documentation

### Objective 
The purpose of this validation is to find the optimal gains for an AFO control system

The metrics used are shown below:
- freq_hz: Initial input signal frequency
- eta, eps: Gains
- Rate of Convergence (s): Time taken to reach within threshold of 0.05 Hz. 
- Max Absolute Error Cadence (Hz): Maximum error
- Mean Average Error Cadence (Hz)
- Root Mean Square Error Cadence (Hz): Punishes large deviations
- Cadence Bias (Hz): Mean of cadence error
- Within Threshold (%): Percentage of time within threshold of 0.05 Hz.
- Initial Frequency (Hz) 
]

In order to easily debug and make sure all parts of the program are working correctly, I've added features incrementally shown as "Version 1,2,3 etc..."  
Note:(All Versions have the previous implementations included)

# Version 1 Adaptive Frequency Oscillator

### Objective
Determine whether difference in input frequency or gain combination is has more of an impact on Mean Average Cadence Error. 

### Gains tested

- eta_values = [1.5, 2, 2.5, 3, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.5, 9.0, 9.5,10]
- eps_values = [1.5, 2, 2.5, 3, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0, 8.5, 9.0, 9.5,10]
- freq_values = [0.5, 0.6, 0.8, 1.1, 1.5, 1.7, 1.9, 2.2]

### Results

See the [adaptive frequency oscillator](Data/afo_validation_results.xlsx).

As shown in sheet "Gain vs Initial Error" I calculated the Mean Average Error Cadence (Hz) for every gain and initial frequency error combination. In order to understand how initial frequency error vs gain combination affects MAE a two factor sum-of-squares decomposition was then used to partition the total variation in cadence MAE into:
1. Variation from gain combination 
Calculated from differences between each gain combinations mean MAE averaged across initial error conditions. Equation shown below:


2. Variation from initial frequency error
Calculated from differences between each initial error conditions mean MAE, averaged across gain combinations. 


## Sum-of-Squares Analysis

Let \(y_{g,f}\) be cadence MAE for gain combination \(g\) and initial-error condition \(f\). Let \(G\) and \(F\) be the numbers of gains and initial-error conditions.

### Means

$$
\bar y_g=\frac{1}{F}\sum_{f=1}^{F}y_{g,f},
\qquad
\bar y_f=\frac{1}{G}\sum_{g=1}^{G}y_{g,f},
\qquad
\bar y=\frac{1}{GF}\sum_{g=1}^{G}\sum_{f=1}^{F}y_{g,f}
$$

### Sum of Squares

$$
SS_{\mathrm{Gain}}
=
F\sum_{g=1}^{G}(\bar y_g-\bar y)^2
$$

$$
SS_{\mathrm{InitialError}}
=
G\sum_{f=1}^{F}(\bar y_f-\bar y)^2
$$

$$
SS_{\mathrm{Total}}
=
\sum_{g=1}^{G}\sum_{f=1}^{F}(y_{g,f}-\bar y)^2
$$

$$
SS_{\mathrm{Residual}}
=
SS_{\mathrm{Total}}
-
SS_{\mathrm{Gain}}
-
SS_{\mathrm{InitialError}}
$$

Therefore:

$$
SS_{\mathrm{Total}}
=
SS_{\mathrm{Gain}}
+
SS_{\mathrm{InitialError}}
+
SS_{\mathrm{Residual}}
$$

### Effect Percentages

$$
\text{Effect}_{i}
=
\frac{SS_i}{SS_{\mathrm{Total}}}\times100\%
$$

| Source | SS | Total variation |
|---|---:|---:|
| Gain combination | 4.318723 | 7.81% |
| Initial-frequency error | 49.402305 | 89.38% |
| Residual/interaction | 1.553066 | 2.81% |
| Total | 55.274093 | 100% |