# iLQR-based CITO
Matlab implementation of contact-implicit trajectory optimization (CITO) based on the variable smooth contact model and iterative linear quadratic regulator (iLQR) for a planar pushing application.

This tool is based on a primitive physics engine that does not consider the friction at the robot-object contacts, self collision, etc. A higher-fidelity C++ implementation is available at https://github.com/aykutonol/cito.

The iLQR implementation is based on https://www.mathworks.com/matlabcentral/fileexchange/52069-ilqg-ddp-trajectory-optimization.

Please consider citing the following papers if you use this code:
* A. Ö. Önol, P. Long and T. Padır, "Contact-Implicit Trajectory Optimization Based on a Variable Smooth Contact Model and Successive Convexification," _2019 IEEE International Conference on Robotics and Automation (ICRA)_. IEEE, 2019. [[arXiv](https://arxiv.org/abs/1810.10462)]
* Tassa, Yuval, Nicolas Mansard, and Emo Todorov. "Control-limited differential dynamic programming." _2014 IEEE International Conference on Robotics and Automation (ICRA)_. IEEE, 2014.
* Li, Weiwei, and Emanuel Todorov. "Iterative linear quadratic regulator design for nonlinear biological movement systems." ICINCO (1). 2004.
