# Position Control of a Rotary Disc (Quanser QUBE-Servo 3)

**EE302: Control Systems Laboratory — Final End-Semester Project (Problem Statement 6)**  
**Group 20**

### Group Members
- **Amey Ballal** (B24328)
- **Vikrant Singh** (B24296)
- **Vipin Nokhwal** (B24297)
- **Vishal Nath** (B24298)

---

## 📌 Project Overview

This project focuses on the modeling, system identification, feedback controller design, simulation, and real-time hardware implementation of a position control system for a rotary disc on the **Quanser QUBE-Servo 3**. 

The main objective is to position the disc accurately at commanded angles (such as $\theta_r = 90^\circ$ and periodic square-wave reference trajectories) with:
- Fast settling time ($t_s$)
- Maximum percentage overshoot **$M_p \le 10\%$**
- Zero steady-state tracking error for step inputs

---

## 📐 Mathematical Modeling

The DC motor driving the rotary disc is modeled using physical principles and transfer function approximations.

### 1. Speed Transfer Function (First-Order)
$$\frac{\Omega(s)}{V(s)} = \frac{K}{\tau s + 1}$$
where:
- $V(s)$: Applied motor armature voltage (V)
- $\Omega(s)$: Angular velocity of the disc (rad/s)
- $K$: Steady-state motor velocity gain (rad/s/V)
- $\tau$: Mechanical time constant (s)

### 2. Position Transfer Function
Integrating angular velocity gives angular position $\Theta(s) = \frac{\Omega(s)}{s}$:
$$G(s) = \frac{\Theta(s)}{V(s)} = \frac{K}{s(\tau s + 1)}$$

### 3. Theoretical Model (Datasheet Parameters)
Using manufacturer physical constants ($R_m = 7.5\ \Omega$, $k_t = k_m = 0.0422\ \text{N}\cdot\text{m/A}$, $J = 1.8299 \times 10^{-5}\ \text{kg}\cdot\text{m}^2$) and neglecting electrical inductance ($L_m \approx 0$) and friction:
$$G_{th}(s) = \frac{23.6967}{s(0.07706 s + 1)}$$
- $K_{th} = 23.6967\ \text{rad/s/V}$
- $\tau_{th} = 0.07706\ \text{s}$

---

## 🔬 System Identification

An open-loop voltage step response experiment was conducted on the physical Quanser QUBE-Servo 3 hardware via QUARC and MATLAB/Simulink:
1. A **$3\text{ V}$ step** was applied through saturation blocks to the motor.
2. Disc position was measured using the optical encoder, converted to radians, and differentiated with a filtered Velocity Estimator to determine angular velocity $\omega(t)$.
3. Experimental response data (`ol (2).mat`) was analyzed using nonlinear least-squares curve fitting (`practice.m`):
   - **Identified Gain ($K$):** $16.294\ \text{rad/s/V}$
   - **Identified Time Constant ($\tau$):** $0.0983\ \text{s}$

### Identified Plant Model
$$G(s) = \frac{16.294}{s(0.0983 s + 1)}$$

### Staircase Validation
To ensure model fidelity across multiple operating voltages, a staircase input profile was generated (`generate_staircase.m`, `staircase.mat`). The identified model accurately predicted speed across all voltage steps with minimal RMSE.

---

## 🎛️ Controller Design

A unity negative feedback loop was designed with a proportional controller gain $K_p$.

### Closed-Loop Transfer Function
$$T(s) = \frac{\Theta(s)}{\Theta_r(s)} = \frac{K_p K}{\tau s^2 + s + K_p K} = \frac{\frac{K_p K}{\tau}}{s^2 + \frac{1}{\tau}s + \frac{K_p K}{\tau}}$$

Comparing with the canonical second-order system $s^2 + 2\zeta\omega_n s + \omega_n^2$:
$$\omega_n = \sqrt{\frac{K_p K}{\tau}}, \qquad 2\zeta\omega_n = \frac{1}{\tau} \implies \zeta = \frac{1}{2\sqrt{K_p K \tau}}$$

### Design Specifications & Gain Selection
- **Overshoot Requirement:** $M_p \le 10\%$
  $$\zeta \ge \frac{-\ln(0.10)}{\sqrt{\pi^2 + \ln^2(0.10)}} \approx 0.591$$
- **Permissible Gain Range:**
  $$K_p = \frac{1}{4 \zeta^2 K \tau}$$
  For underdamped response ($0.591 \le \zeta < 1$):
  $$0.1561 < K_p \le 0.4466$$
- **Chosen Controller Gain:**  
  $$K_p = 0.40$$

### Predicted Closed-Loop Characteristics ($K_p = 0.40$)
- **Natural Frequency ($\omega_n$):** $8.143\ \text{rad/s}$
- **Damping Ratio ($\zeta$):** $0.6247$
- **Predicted Overshoot ($M_p$):** $8.10\%$ ($\le 10\%$)
- **Peak Time ($t_p$):** $0.494\ \text{s}$
- **Settling Time ($t_s$, 2% criterion):** $0.786\ \text{s}$
- **Rise Time ($t_r$):** $0.353\ \text{s}$

---

## 📊 Performance Comparison

| Metric / Parameter | Theoretical (Datasheet) | Simulation (Identified) | Hardware (QUBE-Servo 3) |
| :--- | :---: | :---: | :---: |
| **Gain $K$** (rad/s/V) | $23.6967$ | $16.294$ | Physical Hardware |
| **Time Constant $\tau$** (s) | $0.07706$ | $0.0983$ | Physical Hardware |
| **Natural Frequency $\omega_n$** (rad/s) | $11.094$ | $8.143$ | — |
| **Damping Ratio $\zeta$** | $0.5850$ | $0.6247$ | — |
| **Percentage Overshoot $M_p$** (%) | $10.37\%$ | $8.10\%$ | Minimal / Well-damped |
| **Rise Time $t_r$** (s) | $0.244\ \text{s}$ | $0.353\ \text{s}$ | Slower than simulation |
| **Peak Time $t_p$** (s) | $0.349\ \text{s}$ | $0.494\ \text{s}$ | — |
| **Settling Time $t_s$** (s) | $0.616\ \text{s}$ | $0.786\ \text{s}$ | Slower than simulation |
| **Position Tracking** | Predicted | Excellent | Excellent tracking |

### Key Observations & Discussion
- The proportional controller successfully stabilized the disc and achieved reference position tracking without exceeding the $10\%$ overshoot constraint.
- The physical hardware exhibited slightly slower rise times and smoother transitions compared to ideal simulation. This discrepancy is attributed to:
  1. Unmodeled Coulomb and viscous friction in the motor bearings.
  2. Voltage saturation limits ($\pm 10\text{ V}$) of the amplifier.
  3. Real-time discrete sampling and encoder quantization effects.
- An extended PD controller ($C(s) = K_p + K_d s$) was also implemented and evaluated, demonstrating enhanced derivative damping during step transitions.

---

## 📁 Repository Structure

```text
├── Position Control of a Rotary Disc.pdf               # Lab project problem statement & requirements
├── Position Control of a Rotary Disc_presentation.pdf  # Final presentation slides (Group 20)
├── README.md                                           # Comprehensive project documentation
├── practice.m                                          # System identification & least-squares fitting script
├── generate_staircase.m                                # Staircase signal generation & validation script
├── ol (2).mat                                          # Open-loop step response experimental dataset
├── staircase.mat                                       # Multi-level staircase validation dataset
├── plot1.fig                                           # MATLAB figure: step response / error plots
└── plots.fig                                           # MATLAB figure: system identification & tracking plots
```

---

## 🚀 How to Run

### 1. System Identification
Run the script `practice.m` in MATLAB:
```matlab
practice
```
This loads the `.mat` datasets, performs nonlinear least-squares optimization to compute $K$ and $\tau$, validates against the staircase response, and outputs the comparison metrics and plots.

### 2. Generate Staircase Signal
```matlab
generate_staircase
```

### 3. Real-Time Hardware Control (QUARC)
1. Connect the **Quanser QUBE-Servo 3** via USB.
2. Open the Simulink model configured with QUARC blocks (`HIL Initialize`, `HIL Read Timebase`, `HIL Write Analog`).
3. Set the proportional gain $K_p = 0.4$.
4. Build the model (`Ctrl+B`) and click **Connect to Target** and **Start**.
5. Observe the reference angle and actual disc position on the Scope.
