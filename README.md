Homework 01 - get acquainted with the use of the performance weights wS(s), wU(s) and wT (s) in the denition of the closed loop specications of an armature controlled DC motor linear model.
- Choose the motor parameters, 
- velocity control problem (you want to guarantee that the rotor and the unknown load rotate at steady state at a given angular velocity) and then a position control problem (stop the motor at a given angular position).

Homework 02 - Consider the system described by (eq.) where x1 and x2 represent the positions of the masses m1 and m2 respectively, b is a viscous friction coeficient, k is the elastic coecient of a repulsive spring and f is the control force applied to the
mass m1. This plant is a linear approximation of the system shown in Fig. 1 where a suitcase of mass m2 has been forgotten on the roof of a car. The repulsive spring represents the combined effect of the gravity and the roof curvature, b the relative friction between the roof and the suitcase, m1
is the total mass of the car without the suitcase and f is the driving force. The suitcase is \carried" by the car via the viscous friction force b(x_ 1 - x_ 2). We can consider three possible outputs: 
1. the absolute position of the suitcase (that is x1)
2. the absolute position of the car (that is x2)
3. the relative position of the suitcase with respect to the car (that is x1 - x2).

Homework 03 - Consider the SISO plant G(s) and controller C(s), in the feedback control scheme of Fig. 1. 
1. Considering r and n as inputs, y and m as outputs and setting all the other inputs to 0, the control scheme can be viewed as a 2  2 MIMO system. Write the equivalent closed-loop MIMO transfer function matrix W(s) symbolically for generic G(s) and C(s). For the given
choices of G(s) and C(s), is the corresponding gain matrix (static gain) of W(s) consistent with the closed-loop steady state expected behavior? How do you interpret the input and output directions corresponding to the dierent singular values?
2. Keeping the same inputs r and n, add ey as output (2 inputs - 3 outputs in total), how does the previous analysis change?
3. Similarly, keeping the outputs y and m, add d2 as input (3 inputs - 2 outputs in total), how does the initial analysis change?
4. Consider now a 2  2 system characterized by the transfer function matrix W(s) previously determined; try to formulate a control problem and solve it (if possible).
Problem 2
For the DC-motor you have chosen, explore all (or many) possible control problems involving robustness (stability and performance). For example (just a few ideas): how your previous controller behaves in terms of robust performance, what is the dierence in terms of robust stability making
dierent choices for the uncertainty (structured vs unstructured), design a robustly stabilizing controller for the case of structured uncertainty.
