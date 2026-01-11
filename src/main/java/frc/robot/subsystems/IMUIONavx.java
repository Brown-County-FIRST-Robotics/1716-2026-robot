package frc.robot.subsystems;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.utils.CustomAlerts;
import org.littletonrobotics.junction.Logger;

/** The abstraction for the Kauai labs Navx2 IMU. Moving to use quest instead. */
@Deprecated(forRemoval = true, since = "20250501")
public class IMUIONavx implements IMUIO {
  final AHRS imu;
  final Rotation3d basRotation3d;

  /** Constructs a Navx that uses SPI */
  public IMUIONavx() {
    imu = new AHRS(AHRS.NavXComType.kMXP_SPI);
    var startTime = RobotController.getFPGATime();
    while (!imu.isConnected() && RobotController.getFPGATime() - startTime < 5000000) {
      imu.isConnected();
    }
    basRotation3d = imu.getRotation3d();
    Logger.recordOutput("Firmware/NAVX", imu.getFirmwareVersion());
    CustomAlerts.makeNavxFailAlerts(imu);
  }

  @Override
  public void updateInputs(IMUIOInputs inputs) {
    inputs.tempC = imu.getTempC();
    inputs.rotation =
        new Rotation3d(
                new Quaternion(
                    imu.getQuaternionW(),
                    imu.getQuaternionX(),
                    imu.getQuaternionY(),
                    imu.getQuaternionZ()))
            .minus(basRotation3d);
    inputs.xAccelMPS = imu.getRawAccelX() * 9.8065;
    inputs.yAccelMPS = imu.getRawAccelY() * 9.8065;
    inputs.zAccelMPS = imu.getRawAccelZ() * 9.8065;
  }
}
