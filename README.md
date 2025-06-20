# **Robot Localization and Control Using MPC and EKF**

<div align="center">
  <img src="https://github.com/user-attachments/assets/117da2ed-f335-4a1b-abb7-3360b0f08347" alt="robot_localization" width="500px">
</div>

## Overview
This project demonstrates a robust localization and control system for a differential drive robot within a PyBullet simulation environment using:
- **Model Predictive Control (MPC)** for real-time trajectory tracking and stabilization.
- **Extended Kalman Filter (EKF)** for accurate state estimation under noisy sensor conditions.

Integrating MPC with EKF enables the robot to navigate reliably to target positions while filtering out sensor noise, making the system suitable for real-world applications.

## Project Design and Methodology

### Problem Setup
The robot's goal is to reach and stabilize around a predefined target position. Key components of this setup include:
1. **MPC Controller**: Determines optimal control actions to minimize trajectory deviation while satisfying physical constraints.
2. **EKF Estimator**: Estimates the robot’s position and orientation by filtering noisy sensor data, which is then used by the MPC for enhanced control accuracy.

A **Differential Drive Kinematic Model** simulates the robot’s movement based on wheel velocities, helping to predict its position over time.

### System Workflow

1. **State Estimation with EKF**: The EKF continuously updates the robot's estimated state (position and orientation) based on noisy range-bearing measurements. It predicts the state in each timestep, corrects it with incoming sensor data, and provides this estimated state to the MPC.
2. **Trajectory Control with MPC**: Given the EKF’s estimated state, the MPC calculates optimal control inputs (e.g., linear and angular velocities) to guide the robot towards the target. The MPC minimizes target deviation while accounting for the robot’s constraints and avoiding overshoot.
3. **Real-Time Feedback Loop**: The EKF’s estimates feed continuously into the MPC, creating a feedback loop that enables the robot to adapt to changing conditions and noisy data.

### Running the Code

This project was implemented within a virtual environment, "RoboEnv," provided by the University College London teaching team. As this environment was used for both coursework and assessment, it is not included in this repository. However, it can be found publicly on GitHub [here](https://github.com/VModugno/RoboEnv).

If you wish to run this project and the repository is still accessible, you will need to:
1. Set up the RoboEnv environment.
2. Place the `models` and `configs` folders in the project's running directory.
3. Run:
    ```bash
    python differential_drive_ekf.py
    ```

This repository includes only the code files that implement the core MPC and EKF logic necessary for robotic simulation.

## Results

Below are the simulation results under different conditions.

1. **MPC without EKF (Noise-Free)**: The robot starts at (2, 3) and attempts to stabilize at the target (0, 0) using only the MPC controller in a noise-free environment.

    <div align="center">
      <img src="https://github.com/user-attachments/assets/471b20c4-b7ba-4660-a330-620026cd7401" alt="result_mpct" width="700px">
    </div>

2. **MPC without EKF (With Noise)**: The same configuration is simulated, but with added sensor noise.

    <div align="center">
      <img src="https://github.com/user-attachments/assets/0dab34d2-20e8-456a-a29a-0f81bc729dad" alt="result_mpct_noise" width="700px">
    </div>

    *Under noise, the robot's navigation becomes erratic, highlighting the need for robust estimation.*

3. **MPC with EKF (With Noise)**: Adding the EKF significantly improves the robot's ability to track the target path under noisy conditions.

    <div align="center">
      <img src="https://github.com/user-attachments/assets/624cb15c-9758-482d-bf59-27d8752a38ba" alt="result_mpck_with_noise" width="700px">
    </div>

    *Using 20 landmarks in a square boundary, the robot successfully reaches the goal with improved trajectory estimation.*


4. **Testing with Various Starting Points**: Different initial positions were used to test robustness.

   <div align="center">
     <img src="https://github.com/user-attachments/assets/41377845-01e2-4526-a65b-9b1ee40c97a3" alt="result_mpck_tests" width="900px">
   </div>

   *Evaluation metrics for various starting points, including steady-state error, overshoot, and settling time, are shown below:*

   <div align="center">
     <img src="https://github.com/user-attachments/assets/8fd78539-ad11-4fc7-a2f4-b719ad56fe24" alt="evaluation_metrics" width="348px">
   </div>


Overall, these results demonstrate that integrating an EKF with MPC significantly enhances localization and control robustness under noisy conditions.

## License

This project is licensed under the GNU GENERAL PUBLIC LICENSE v3.0. See the [LICENSE](LICENSE) file for more details.

## Acknowledgments

- This project was developed as part of the assessment for the [COMP0242 - Estimation and Control](https://www.ucl.ac.uk/module-catalogue/modules/estimation-and-control-COMP0242) module at University College London. 
- Special thanks to the teaching team for providing the simulation environment and guidance throughout the project.

