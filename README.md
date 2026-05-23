# Embedded-Distributed-Optimization-ADMM
Implementation of an Embedded Accelerated Decentralized Optimization Framework for Renewable Energy Communities (RECs). This project features a ADMM algorithm tailored for real-time energy management on resource-constrained embedded platforms, maximizing shared energy while ensuring privacy-preserving individual cost minimization.
# Embedded Accelerated Distributed Optimization for Energy Communities

## Overview
This repository provides a high-performance simulation framework for the optimal management of **Renewable Energy Communities (RECs)**. The project implements a decentralized optimization architecture that balances the collective goal of maximizing shared energy with the individual objective of minimizing prosumer costs.

The core of the solver is a **Nesterov-accelerated Alternating Direction Method of Multipliers (ADMM)**, specifically reformulated for embedded implementation on low-power microcontrollers.

## Technical Highlights
- **Bi-level to Single-level Reformulation**: The interaction between the Energy Community Manager (ECM) and participants (ECPs) is modeled as a bi-level program, converted into a single-level Quadratic Programming (QP) problem using KKT optimality conditions.
- **Decentralized Execution**: The problem is fully decomposed across participants, ensuring data privacy and eliminating the need for a central coordinator to exchange sensitive local variables.
- **Accelerated Convergence**: Adopts Nesterov-type acceleration for both primal and dual updates.
- **Real-time Feasibility**: Designed for resource-constrained hardware by exploiting matrix sparsity and pre-computing invariant operators to ensure deterministic execution times.

## Numerical Observations and Discussion
- **On Convergence Performance Observation**: In this specific REC scheduling model, the Fast ADMM variant did not demonstrate a significantly higher convergence speed compared to the classical ADMM.   Analysis: This is likely due to the model's scale and its strong convexity. For small-scale problems ($N=3, T=12$), the classical ADMM can reach high precision rapidly within the linear convergence region. Furthermore, the introduction of a quadratic penalty term ensures the objective function is strongly convex, which allows standard first-order methods to perform exceptionally well, thereby reducing the marginal gain of Nesterov acceleration.
- **On Residual Non-Zero LimitObservation**: The primal and dual residuals reached a plateau and did not converge to absolute zero.   Analysis: This phenomenon may be attributed to the KKT-based reformulation, a topic reserved for future analysis using adaptive penalty methods.

## References
[1] G. Ferro, S. Grammatico, L. Parodi, R. R. Baghbadorani, and M. Robba, "An embedded accelerated decentralized optimization algorithm with application to energy communities," Control Engineering Practice, vol. 172, p. 106920, Mar. 2026.  
[2] T. Goldstein, B. O'Donoghue, S. Setzer, and R. Baraniuk, "Fast Alternating Direction Optimization Methods," SIAM Journal on Imaging Sciences, vol. 7, no. 3, pp. 1588–1623, 2014.  
