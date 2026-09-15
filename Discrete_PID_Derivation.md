# Overview
While real-life systems exist in the continuous time domain, PID controllers are often discretised for software implementation because doing so:
- Ensures compatibility with digital hardware (e.g. microcontrollers) which perform computations in discrete time steps.
- Makes the algorithm more robust against noise.
- Reduces computational load.

A typical PID control loop looks like this:

![Typical PID control loop](https://github.com/ben-boozled/PID_Controller/blob/main/Images/PID_Plant.png)

The transfer function for the control loop above is: 

$$
\begin{aligned}
G(s) = \frac{U(s)}{E(s)} = K_{p} + \frac{K_{i}}{s}  + \frac{K_{d}s}{1 + T_{f}s}
\end{aligned}
$$

* $U(s)$: controller output.
* $E(s)$: error input.
* ${T_{f}}$: filter time constant in seconds. Note that $\frac{1}{T_{f}}$ is the low-pass filter's cutoff frequency in rad/s.

# Discrete PID Derivation (Tustin Transform)
There several ways to  discretise the continuous time PID equations. I chose the Tustin Transform because the resultant controller is generally more stable and has second order accuracy. Other discretising methods include the Forward Euler and Backward Euler methods though I will not be covering them here.

According to the [Tustin's Approximation](https://en.wikipedia.org/wiki/Bilinear_transform):

$$
\begin{aligned}
z=e^{s_T{}} = \frac{e^{\frac{sT_{s}}{2}}}{e^{\frac{-sT_{s}}{2}}} \approx \frac{1+\frac{sT}{2}}{1-\frac{sT}{2}} \\
s \approx \frac{2}{T_{s}}·\frac{z-1}{z+1}
\end{aligned}
$$

Where $T_{s}$ is the sampling time.

By substituting the above result for $s$, we can map the controller's transfer function from the $s$-plane (which is used for continuous time analog signals) to the $z$-plane (which is used for discrete time digital signals).

## Deriving the Difference Equations
$u[n]$, the output of the PID controller, can be interpreted as a sum of the individual contributions from its proportional, integral, and derivative subsystems:

$$
\begin{aligned}
u[n] = u_{p}[n] + u_{i}[n] + u_{d}[n]
\end{aligned}
$$

The subsequent sections will solve the difference equations for each of the above components. Separating the contributions from each controller subsystem is advantageous; we can evaluate and address **integral windup** and **derivative kick** issues for the integral and derivative components respectively. 

Additionally, I will be following the framework below to convert the $z$-domain equations (of the form $\frac{U(z)}{E(z)}=\frac{N(z)}{D(z)}$) into difference equations:
1. Cross multiply: ${D(z)}·{U(z)}={N(z)}·{E(z)}$.
2. Expand the powers of $z$.
3. Divide the equation by the highest powered $z$.
4. Apply the relationship: $z^{-k}·Y(z)=y[n-k]$.

### Proportional Term
$$
\begin{aligned}
\frac{U_{p}(s)}{E(s)} = K_{p} \\
\frac{U_{p}(z)}{E(z)} = K_{p} \\
U_{p}(z) = K_{p}E(z) \\
u_{p}[n] = K_{p}e[n]
\end{aligned}
$$

### Integral Term
$$
\begin{aligned}
\frac{U_{i}(s)}{E(s)} = \frac{K_{i}T_{s}}{s}\\
\frac{U_{i}(z)}{E(z)} = \frac{K_{i}T_{s}}{2}·\frac{z+1}{z-1}\\
U_{i}(z)·z - U_{i}(z) = \frac{K_{i}T_{s}}{2}·[E(z)·z+E(z)]\\
U_{i}(z) - z^{-1}·U_{i}(z) = \frac{K_{i}T_{s}}{2}·[E(z)·+z^{-1}·E(z)]\\
u_{i}[n] - u_{i}[n-1] = \frac{K_{i}T_{s}}{2}·[e[n]+e[n-1]]\\
u_{i}[n] = \frac{K_{i}T_{s}}{2}·[e[n]+e[n-1]] + u_{i}[n-1]
\end{aligned}
$$

_Integral Windup_

An integral windup is a situation where the integral term accumulates large errors. This can occur if, for instance, the demands from the controller's signal exceeds the actuator's physical limits (e.g. commanding a motorised ball valve to open beyond 100%). While the actuator's performance is clamped by inherent physical limits, the integral term continues accumulating errors. This can lead to large system overshoots and long settling times as the controller needs time to correct the accumulated errors.

To address this, the PID implementation here includes an integral clamping feature. If the total control signal is saturated AND the integral term is worsening the saturation (i.e. increasing the control signal when the system is at its maximum limit or vice versa), the integral term will be frozen to its value in the previous time step:

$$
\begin{aligned}
u_{i}[n] = u_{i}[n-1]  
\end{aligned}
$$

### Derivative Term
$$
\begin{aligned}
\frac{U_{d}(s)}{E(s)} = \frac{K_{d}s}{1+T_{f}s}\\
\frac{U_{d}(z)}{E(z)} = \beta·\frac{z-1}{z+\alpha},\ \alpha = \frac{T-2T_{s}}{T+2T_{f}},\ \beta = \frac{2K_{d}}{T_{s}+2T_{f}}\\
U_{d}(z)·(z+\alpha) = E(z)·\beta·(z-1)\\
U_{d}(z) + \alpha·z^{-1}U_{d}(z) = \beta·E(z) - \beta·z^{-1}E(z)\\
u_{d}[n] = \beta·[e[n]-e[n-1]] - \alpha·u_{d}[n-1]
\end{aligned}
$$

_Derivative Kick_

In a standard PID controller, $e(t) = SP - PV$ where $e(t)$ is the error term, $SP$ the setpoint, and $PV$ the process variable. Any setpoint changes can engender sudden spikes in the control signal, which might cause the actuator to operate in a manner that damages itself. To address this, the derivative component should be calculated based on the process variable. This approach is known as "derivative on measurement".

For a constant setpoint,

$$
\begin{aligned}
\frac{de(t)}{dt} = \frac{d(SP-PV)}{dt}\\
\frac{de(t)}{dt} = -\frac{dPV}{dt} \\
\end{aligned}
$$

Hence, from time step $t-1$ to $t$,

$$
\begin{aligned}
e(t)-e(t-1) = -[PV(t)-PV(t-1)]
\end{aligned}
$$

Using the above relationship, we obtain a new result for $u_{d}[n]$ that is based on the process variable:

$$
\begin{aligned}
u_{d}[n] = -\beta·[PV[n]-PV[n-1]] - \alpha·u_{d}[n-1]
\end{aligned}
$$

The **negative sign in front of** $\mathbf{\beta}$ is absolutely crucial. ~~This me speaking from the experience of questioning my life choices after spending hours debugging the PID output because it kept oscillating endlessly till the end of time~~.

## Final Result
Consolidating the above derivations, the key equations to implement the discrete time PID are as follows:

$$
\begin{aligned}
u[n] = u_{p}[n] + u_{i}[n] + u_{d}[n]\\
u_{p}[n] = K_{p}e[n]\\
u_{i}[n] = \frac{K_{i}T_{s}}{2}·[e[n]+e[n-1]] + u_{i}[n-1]\\
u_{d}[n] = -\beta·[PV[n]-PV[n-1]] - \alpha·u_{d}[n-1],\ \alpha = \frac{T-2T_{s}}{T+2T_{f}},\ \beta = \frac{2K_{d}}{T_{s}+2T_{f}}
\end{aligned}
$$

