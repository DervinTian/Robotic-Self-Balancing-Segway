# Robotic-Self-Balancing-Segway

Task: Given a two-wheeled Segway, our task was to allow for autonomous self-balancing of the segway. All code is performed through Arduino and Matlab.

Actions taken: Using Model Predictive Control (MPC), we first started by creating a model using MATLAB Simulink to create a model to predict controller parameters. The controller was used to control the motor and segway movements. To allow for self adjusting, the Kalman Filtering technique was used in order to predict future states using previous states in a postiive feedback loop. Everything was coded in Arduino and the segway operated using the Kalman Filter to predict future states and fed back into the controller in order to self-correct.

Results: Segway was able to balance upright for a couple of seconds. Results were were also influcenced by outside factors which made the performance less than optimal. Problems with proper motor function limited the capabilities of the code, but worked well on high-functioning segways.
