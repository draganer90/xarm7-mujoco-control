# References and theoretical basis

This repository documents a simulation-based study of kinematics, trajectory planning and control for the seven-degree-of-freedom xArm7 manipulator using MuJoCo and Pinocchio.

**Primary theoretical reference:** Siciliano et al. [1]. Spong et al. [2] provides complementary coverage. Peer-reviewed journal articles [3]-[5] supply foundational research context. Software publications [6]-[7] acknowledge the scientific tools used by the project.

## Core textbooks

[1] B. Siciliano, L. Sciavicco, L. Villani, and G. Oriolo, *Robotics: Modelling, Planning and Control*. Springer London, 2009. DOI: [10.1007/978-1-84628-642-1](https://doi.org/10.1007/978-1-84628-642-1).

[2] M. W. Spong, S. Hutchinson, and M. Vidyasagar, *Robot Modeling and Control*, 2nd ed. John Wiley & Sons, 2020. Print ISBN: 978-1-119-52399-4.

## Peer-reviewed journal literature

[3] O. Khatib, "A unified approach for motion and force control of robot manipulators: The operational space formulation," *IEEE Journal on Robotics and Automation*, vol. 3, no. 1, pp. 43-53, 1987. DOI: [10.1109/JRA.1987.1087068](https://doi.org/10.1109/JRA.1987.1087068).

[4] B. Armstrong-Hélouvry, P. Dupont, and C. Canudas de Wit, "A survey of models, analysis tools and compensation methods for the control of machines with friction," *Automatica*, vol. 30, no. 7, pp. 1083-1138, 1994. DOI: [10.1016/0005-1098(94)90209-7](https://doi.org/10.1016/0005-1098(94)90209-7).

[5] T. Yoshikawa, "Manipulability of robotic mechanisms," *The International Journal of Robotics Research*, vol. 4, no. 2, pp. 3-9, 1985. DOI: [10.1177/027836498500400201](https://doi.org/10.1177/027836498500400201).

## Simulation and dynamics software publications (conference papers)

[6] E. Todorov, T. Erez, and Y. Tassa, "MuJoCo: A physics engine for model-based control," in *2012 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp. 5026-5033, 2012. DOI: [10.1109/IROS.2012.6386109](https://doi.org/10.1109/IROS.2012.6386109).

[7] J. Carpentier, G. Saurel, G. Buondonno, J. Mirabel, F. Lamiraux, O. Stasse, and N. Mansard, "The Pinocchio C++ library -- A fast and flexible implementation of rigid body dynamics algorithms and their analytical derivatives," in *IEEE International Symposium on System Integrations (SII)*, 2019. Official software repository: [stack-of-tasks/pinocchio](https://github.com/stack-of-tasks/pinocchio).

## Mapping sources to the implemented methods

| Project topic | Recommended source | Scope of support |
| --- | --- | --- |
| Forward/inverse kinematics; geometric Jacobian; redundancy | [1] Chapters 2-3; [2] Chapters 3-5 | Classical manipulator kinematics and numerical IK |
| Quintic reference profiles; joint and Cartesian path planning | [1] Chapter 4; [2] Chapter 7 | Planning and time parameterization |
| Manipulator dynamics; inverse dynamics | [1] Chapter 7; [2] Chapter 6 | Equations of motion and numerical dynamics |
| PD plus gravity; computed torque; task-space control | [1] Chapter 8; [2] Chapters 8-9; [3] | Classical control laws and operational-space formulation |
| Singularities and redundancy interpretation | [1] Chapter 3; [5] | Jacobian conditioning and manipulability |
| Dry friction diagnostics and limitations of feedforward compensation | [4] | Friction modeling and compensation challenges |
| MuJoCo and Pinocchio software attribution | [6] and [7] | Software methods and provenance |

**Academic interpretation:** [3] gives foundational context for operational-space control; it must not be cited as proof that this repository implements Khatib's exact operational-space controller. Likewise, [4] contextualizes friction effects but does not validate this repository's specific predicted-velocity feedforward heuristic. The project's claims about accuracy and stability must be supported by its own recorded experiments and explicitly stated simulation conditions.

**Reproduction and redistribution:** Cite these publications using bibliographic entries and links. Do not commit full copyrighted textbook PDFs or publisher article PDFs to the public repository. Citation is not equivalent to permission to redistribute documents.
