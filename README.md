TITLE:
Controle Preditivo Não-Linear Aplicado ao Sistema de Piloto Automático de Mísseis

ABSTRACT:
This dissertation addresses the development and application of Model-Based Predictive
Control (MPC) in missile autopilot systems. Detailed mathematical modeling of axisym-
metric missiles controlled by fins is presented, including practical simplifications that
maintain the necessary precision for effective control throughout the flight envelope. Based
on this modeling, a Nonlinear MPC controller is developed and tested through simulations,
comparing its performance with a controller previously designed by Simões and Cavalcanti
(1) ( 2), highlighting the feasibility and efficacy of MPC in complex and nonlinear appli-
cations like missile autopilot control. The simplicity, intuitiveness, and precision of the
MPC, especially when applied to systems with well-defined mathematical modeling, are
emphasized. The performance of the MPC controller proves superior in comparison to the
IQC controller in nominal simulations, exhibiting robust stability to parametric variations
and the capability of real-time implementation, given the rapid optimization solutions
achieved. Significant contributions include the formulation of an approximate equation for
dynamic pressure in compressible fluids, for different Mach numbers and altitudes, and the
proposition of an MPC controller with two internal models, one for prediction and another
for state updates. Identified limitations pave the way for future research, suggesting the
inclusion of robust control techniques in MPC, the application of more realistic missile
aerodynamic data, and the consideration of the complete 6DOF dynamics. This work not
only advances knowledge in the field of Nominal MPC control but also establishes a solid
foundation for future research in the area of Nonlinear and Robust MPC control systems.

Keywords: mpc. nonlinear. missile. 3dof. autopilot. matlab. casadi.

## Software Configuration

| Software                | Version    |
|-------------------------|------------|
| Matlab                  | R2023a     |
| CasADi                  | 3.6.3      |
| Mosek                   | 10.0       |
| Microsoft Visual Studio | 2022       |

## Hardware Configuration

| Component           | Specification          |
|---------------------|------------------------|
| Processor           | Core i5-1235U          |
| L3 Cache            | 12 MB                  |
| Base Frequency      | 1.30 GHz               |
| Maximum Frequency   | 4.40 GHz               |
| Cores               | 10                     |
| Threads             | 12                     |
| TDP Max             | 15 W                   |
| Integrated Graphics | Intel Iris Xe Graphics |
| Total Memory        | 16 GB                  |
| Memory Type         | LPDDR4x                |
| Memory Speed        | 4267 MHz               |
