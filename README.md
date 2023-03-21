# Battery Monitoring System
1. "Mar_16_2023_Arduino_NodeMCU" folder:
- It contains code for running Coulomb Counting only. However, this program 
enable communication between UNO and nodeMCU, therefore, data can be uploaded to Blynk
- It cannot run Kalman Filter since Arduino UNO is not capable of handling large number 

2. "Kalman Filter" folder:
- As its name suggests, it is used to run Kalman Filter (KF). Arduino UNO collects sensor data and PC (Python)
runs the algorithm.
- Since we realized that Arduino UNO could not run KF only more than a week before the deadline, we didn't have enough
time add IoT features to this program. For anyone interested in this project, we suggest Raspberry Pi or STM32 as your MCU
for IoT applications.
