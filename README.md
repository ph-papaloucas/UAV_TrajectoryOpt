# HOW TO BUILD ME 

Relative paths assume executables are placed in ./build folder

```sh
mkdir build
cd build
cmake ..
cmake --build .
```


# Aerodynamic Stability-Derivative Model

## Longitudinal (symmetric) coefficients

Lift coefficient:

$$
C_L = C_{L0} + C_{L_\alpha}\,\alpha
      + C_{L_q}\,\frac{\bar c\,q}{2V}
      + C_{L_{\delta_e}}\,\delta_e
$$

Drag coefficient:

$$
C_D = C_{D0} + C_{D_\alpha}\,\alpha
      + C_{D_q}\,\frac{\bar c\,q}{2V}
      + C_{D_{\delta_e}}\,\delta_e
$$

Pitching-moment coefficient:

$$
C_m = C_{m0} + C_{m_\alpha}\,\alpha
      + C_{m_q}\,\frac{\bar c\,q}{2V}
      + C_{m_{\delta_e}}\,\delta_e
$$

---

## Lateral/Directional coefficients

Side-force coefficient:

$$
C_Y = C_{Y0} + C_{Y_\beta}\,\beta
      + C_{Y_p}\,\frac{b\,p}{2V}
      + C_{Y_r}\,\frac{b\,r}{2V}
      + C_{Y_{\delta_a}}\,\delta_a
      + C_{Y_{\delta_r}}\,\delta_r
$$

Rolling-moment coefficient:

$$
C_l = C_{l0} + C_{l_\beta}\,\beta
      + C_{l_p}\,\frac{b\,p}{2V}
      + C_{l_r}\,\frac{b\,r}{2V}
      + C_{l_{\delta_a}}\,\delta_a
      + C_{l_{\delta_r}}\,\delta_r
$$

Yawing-moment coefficient:

$$
C_n = C_{n0} + C_{n_\beta}\,\beta
      + C_{n_p}\,\frac{b\,p}{2V}
      + C_{n_r}\,\frac{b\,r}{2V}
      + C_{n_{\delta_a}}\,\delta_a
      + C_{n_{\delta_r}}\,\delta_r
$$

---

## Dimensional Forces & Moments

Dynamic pressure:

$$
q_\infty = \tfrac{1}{2}\rho V^2
$$

Forces:

$$
L = q_\infty S\, C_L,\qquad
D = q_\infty S\, C_D,\qquad
Y = q_\infty S\, C_Y
$$

Moments:

$$
M = q_\infty S \bar c\, C_m
$$

$$
L_{roll} = q_\infty S b\, C_l,\qquad
N_{yaw} = q_\infty S b\, C_n
$$

---

## Definitions

- $\alpha$ — angle of attack (rad)  
- $\beta$ — sideslip angle (rad)  
- $p,q,r$ — roll, pitch, yaw rates (rad/s)  
- $\bar c$ — mean aerodynamic chord  
- $b$ — wingspan  
- $V$ — true airspeed (m/s)  
- $\rho$ — air density  
- $S$ — wing reference area  

Nondimensional rate scalings:  
- $\dfrac{\bar c q}{2V}$ (pitch rate)  
- $\dfrac{b p}{2V}$ (roll rate)  
- $\dfrac{b r}{2V}$ (yaw rate)  


# Improvements 

Simulation: cache forces of RK4 last timestep at step3, to reuse for RK4 next timestep at step0. Difficult to generalize this for any integration_method, so it should be applied only when RK4 is used
