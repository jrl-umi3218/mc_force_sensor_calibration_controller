Force Sensor Calibration
==

Requirements
==

Running this controller require to install the [Ceres library](https://github.com/ceres-solver/ceres-solver)

This controller allows to calibrate robot force sensors to allow `mc_rtc` to remove the effect of gravity due to links attached to the force sensors (grippers/feet).

Robots currently supported:
- `HRP4LIRMM`
- `HRP4J`
- `HRP5P`
- `JVRC1`
- `panda_default`

To add your own robot, define the appropriate configuration in `etc/ForceSensorCalibration.in.yaml` and sumbit a pull request.

How to use
==

To calibrate, simply run this controller and follow the instructions in the GUI.

```yaml
MainRobot: JVRC1 # one of the supported robots
Enabled: ForceSensorCalibration
```

- As a safety, the controller will first check that none of the sensor readings are above a specified threshold (ensures that floating base robots are in the air)
  - If the test fails, it'll display a message in the GUI. Click on `Continue` once the robot is in the air
- Then you will be presented with a `Calibration` tab in the GUI:
  - Start calibration: perform the calibration motion (going to an initial posture, and making each sensor move simultaneously. Once the motion is completed, it'll run the calibration optimization, and move to the next `Check calibration` state
  - Check calibration: loads the calibration results, perform the calibration motion again and displays live plots of the results. The calibrated force is expected to be close to `0N`. You can either save the current calibration results if you are satisfied using `Save calibration` or `Save and finish`, or stop without keeping the calibration results using `Finish without saving`
  - Once the check calibration state is finished, the robot will go back to halfsitting.
  - If there is an error, the robot will go back to halfsitting.

The expected result in the live plots should look similar to:

![Example calibration result for HRP4](doc/hrp4_calibration_example.png)


Note for simulation
==

For floating base robots, you need to have the robot in the air. In choreonoid, this can be achieved by changing the root joint type to `fixed` in the robot's `wrl` model.
For convenience, we provide `*_fixed.cnoid` variants of our main robots (`JVRC1`, `HRP4LIRMM`, `HRP4J`, `HRP5P`), use these variants to run the calibration controller in simulation.

TroubleShooting
==

*HRP4*:

Currently the real HRP4 has a force sensor with reading flipped along one of its axis. As a result, the choreonoid simulation and reality must be configured differently:

- Choreonoid:
  - Use `*_fixed.cnoid`
  - Don't forget to use the robot module `HRP4ComanoidChoreonoid` to have the correct force sensor frames

- Real
  - Use `HRP4Comanoid` robot module
