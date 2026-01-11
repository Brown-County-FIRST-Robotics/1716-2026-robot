package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Temperature;

/** The abstraction for the CTRE Pigeon 2 IMU. Moving to use quest instead. */
@Deprecated(forRemoval = true, since = "20250501")
public class IMUIOPigeon implements IMUIO {
  final Pigeon2 imu;
  final StatusSignal<Double> qW;
  final StatusSignal<Double> qX;
  final StatusSignal<Double> qY;
  final StatusSignal<Double> qZ;
  final StatusSignal<LinearAcceleration> accelX;
  final StatusSignal<LinearAcceleration> accelY;
  final StatusSignal<LinearAcceleration> accelZ;
  final StatusSignal<Temperature> temp;

  /**
   * Constructs an IMU using a CAN id
   *
   * @param id The CAN id of the pigeon2
   */
  public IMUIOPigeon(int id) {
    imu = new Pigeon2(id);
    qW = imu.getQuatW();
    qX = imu.getQuatX();
    qY = imu.getQuatY();
    qZ = imu.getQuatZ();
    accelX = imu.getAccelerationX();
    accelY = imu.getAccelerationY();
    accelZ = imu.getAccelerationZ();
    temp = imu.getTemperature();
    qW.setUpdateFrequency(50.0);
    qX.setUpdateFrequency(50.0);
    qY.setUpdateFrequency(50.0);
    qZ.setUpdateFrequency(50.0);
    accelX.setUpdateFrequency(50.0);
    accelY.setUpdateFrequency(50.0);
    accelZ.setUpdateFrequency(50.0);
    temp.setUpdateFrequency(20.0);
    imu.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IMUIOInputs inputs) {
    BaseStatusSignal.refreshAll(temp, qW, qX, qY, qZ, accelX, accelY, accelZ);
    inputs.tempC = temp.getValue().in(Units.Celsius);
    inputs.rotation =
        new Rotation3d(new Quaternion(qW.getValue(), qX.getValue(), qY.getValue(), qZ.getValue()));
    inputs.xAccelMPS = accelX.getValue().in(Units.MetersPerSecondPerSecond);
    inputs.yAccelMPS = accelY.getValue().in(Units.MetersPerSecondPerSecond);
    inputs.zAccelMPS = accelZ.getValue().in(Units.MetersPerSecondPerSecond);
  }
}
