# Feedforward Validation Documentation

### Objective 
The purpose of this validation is to find the optimal gains for an AFO control system

The metrics used are shown below:

columns = [
    "score",
    "freq_hz",
    "eta",
    "eps",
    "Rate of Convergence (s)",
    "Max Absolute Error Cadence (Hz)",
    "Mean Average Error Cadence (Hz)",
    "Root Mean Square Error Cadence (Hz)",
    'Cadence Bias (Hz)',
    "Within Threshold (%)",
    "Initial Frequency Error (Hz)",
]




## Sum of Squares Analysis

Let:

- \(y_{g,f}\) be the cadence MAE for gain combination \(g\) under initial-frequency-error condition \(f\).
- \(G\) be the number of gain combinations.
- \(F\) be the number of initial-frequency-error conditions.
- \(\bar{y}_g\) be the average MAE for gain combination \(g\).
- \(\bar{y}_f\) be the average MAE for frequency condition \(f\).
- \(\bar{y}\) be the grand mean across all observations.

### Mean MAE for Each Gain Combination

\[
\bar{y}_g
=
\frac{1}{F}
\sum_{f=1}^{F} y_{g,f}
\]

### Mean MAE for Each Initial-Frequency-Error Condition

\[
\bar{y}_f
=
\frac{1}{G}
\sum_{g=1}^{G} y_{g,f}
\]

### Grand Mean

\[
\bar{y}
=
\frac{1}{GF}
\sum_{g=1}^{G}
\sum_{f=1}^{F}
y_{g,f}
\]

### Gain-Combination Sum of Squares

\[
SS_{\text{Gain}}
=
F
\sum_{g=1}^{G}
\left(
\bar{y}_g-\bar{y}
\right)^2
\]

### Initial-Frequency-Error Sum of Squares

\[
SS_{\text{Initial Error}}
=
G
\sum_{f=1}^{F}
\left(
\bar{y}_f-\bar{y}
\right)^2
\]

### Total Sum of Squares

\[
SS_{\text{Total}}
=
\sum_{g=1}^{G}
\sum_{f=1}^{F}
\left(
y_{g,f}-\bar{y}
\right)^2
\]

### Residual and Interaction Sum of Squares

\[
SS_{\text{Residual}}
=
SS_{\text{Total}}
-
SS_{\text{Gain}}
-
SS_{\text{Initial Error}}
\]

Equivalently:

\[
SS_{\text{Residual}}
=
\sum_{g=1}^{G}
\sum_{f=1}^{F}
\left(
y_{g,f}
-
\bar{y}_g
-
\bar{y}_f
+
\bar{y}
\right)^2
\]

The sum-of-squares components satisfy:

\[
SS_{\text{Total}}
=
SS_{\text{Gain}}
+
SS_{\text{Initial Error}}
+
SS_{\text{Residual}}
\]

### Effect Percentages

\[
\text{Gain Effect}
=
\frac{SS_{\text{Gain}}}
{SS_{\text{Total}}}
\times 100\%
\]

\[
\text{Initial-Error Effect}
=
\frac{SS_{\text{Initial Error}}}
{SS_{\text{Total}}}
\times 100\%
\]

\[
\text{Residual Effect}
=
\frac{SS_{\text{Residual}}}
{SS_{\text{Total}}}
\times 100\%
\]

For the stable gain combinations included in the analysis:

\[
SS_{\text{Gain}} = 4.318723
\]

\[
SS_{\text{Initial Error}} = 49.402305
\]

\[
SS_{\text{Residual}} = 1.553066
\]

\[
SS_{\text{Total}} = 55.274093
\]

Therefore:

\[
\text{Gain Effect} = 7.81\%
\]

\[
\text{Initial-Error Effect} = 89.38\%
\]

\[
\text{Residual Effect} = 2.81\%
\]