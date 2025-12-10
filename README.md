# IR project work

## Group27

This work addresses the problem of unstable behavior switching and efficiency degradation commonly observed in mobile robots operating in cluttered environments. To improve stability and robustness, we propose a hybrid navigation and decision-making framework that integrates fuzzy activation, hysteresis-based jitter suppression, and a Vector Field Histogram (VFH) inspired local planner within a behavior tree architecture. Fuzzy activation smooths the decision boundaries by providing continuous behavior relevance scores, while the hysteresis and cooldown mechanism effectively reduces rapid switching and mitigates oscillatory behavior. The VFH-style local planner further enhances obstacle avoidance, particularly in dense or irregular environments. We evaluate the proposed system in three representative Webots simulation scenarios and compare its performance against a baseline behavior tree controller. Experimental results show that our approach significantly increases goal-reaching success rates, reduces behavior switching frequency, and improves overall path smoothness and navigation stability. Although failures still occur in narrow U-shaped corridors due to the limitations of purely reactive local planning, the results demonstrate the effectiveness of the proposed framework in enhancing the reliability and consistency of robot navigation.

Project Description: 
The "analysis" folder is used for conducting data analysis. It stores the output data generated during the code execution and uses Python to perform graphical analysis on it. 
The "controllers" folder contains the core code that controls the movement of the robot. 
The "Worlds" is the code for the Webot world. 
The other txt files are all auxiliary files.

Group manber: Username  Real name
cwjgiegie - Wenjian Chen
littleHandsomeboy - Wenjun He
xc-LEUNG - Xiaocong Liang
Duochuan9 - Hongbo Han
