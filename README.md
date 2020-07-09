Force Sensor Calibration
==

Requirements
==

Running this controller require to install the [Ceres library](https://github.com/ceres-solver/ceres-solver)

**IMPORTANT WARNING: This controller must be run with the robot in the air as it'll move the feet around**

This controller allows to calibrate the force sensors of HRP robots to remove the effect of gravity due to links attached to the force sensors (grippers/feet).

To calibrate, simply run this controller and follow the instructions in the GUI.
```yaml
Enabled: ForceSensorCalibration
```

**IMPORTANT WARNING: This controller must be run with the robot in the air as it'll move the feet around**

If all goes well, it will:
- make the joints move in periodic motions
- run the calibration script: this may take a while (up to a few minutes). Be patient, the robot will start moving as soon as it is finished.
- if the previous state succeeds, the controller then performs a calibration check which requires user input:
  - Performs the same calibration motion
  - Display live plots with the raw and calibrated measurements. Check that the calibrated measurements are all close to zero. If that's the case, calibration was successful, and you should click on *Save calibration* in the GUI if you wish to make it the new default calibration.

![Example calibration result for HRP4](doc/hrp4_calibration_example.png)


Note for simulation
==

If you wish to calibrate in simulation in choreonoid, this is a bit problematic as the feet are on the ground.

You may either:
- Find a way to have the robot in the air during a dynamic simulation (if you know how, please let me know).
- Comment out the leg joints in the configuration file and only calibrate the grippers.

TroubleShooting
==

*HRP4*:

Currently the real HRP4 has a force sensor with reading flipped along one of its axis. As a result, the choreonoid simulation and reality must be configured differently:

- Choreonoid:
  - Use *comanoid.cnoid
  - Don't forget to use the robot module `HRP4ComanoidChoreonoid` to have the correct force sensor frames 

- Real
  - Use `HRP4Comanoid` robot module
