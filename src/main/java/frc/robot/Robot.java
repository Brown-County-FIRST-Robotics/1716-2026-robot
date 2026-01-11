package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.Alert;
import frc.robot.utils.CustomAlerts;
import frc.robot.utils.PeriodicRunnable;
import java.io.File;
import java.io.FileNotFoundException;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;
import java.util.Scanner;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  boolean builtPoseSetter = false;
  private RobotContainer robotContainer;

  private final XboxController driverController = new XboxController(0);
  private final boolean hasRumbledMatchTime = false; // hasStarted, hasEnded

  @Override
  public void robotInit() {
    var capture = CameraServer.startAutomaticCapture();
    Shuffleboard.getTab("Teleop").add(capture).withSize(6, 5).withPosition(3, 0);
    // Record metadata
    Logger.recordMetadata("ProjectName", "2025");
    File deployDir = Filesystem.getDeployDirectory();
    File tagFile = new File(deployDir, "git_tag.txt");
    File deployerFile = new File(deployDir, "deployer.txt");
    String tagName;
    String deployer;
    try {
      Scanner reader = new Scanner(tagFile);
      tagName = reader.nextLine();
      reader.close();
    } catch (FileNotFoundException e) {
      tagName = "Deploy did not send git data";
      new Alert(
              "Git data was not included in deploy. This will make it impossible to determine what code was run from the logfile. ",
              Alert.AlertType.WARNING)
          .set(true);
    }
    try {
      Scanner reader = new Scanner(deployerFile);
      deployer = reader.nextLine();
      reader.close();
    } catch (FileNotFoundException e) {
      new Alert("The identity of the deployer is unknown", Alert.AlertType.WARNING).set(true);
      deployer = "Unknown deployer";
    }
    Logger.recordMetadata("Tag Name", tagName);
    Logger.recordMetadata("Deployer", deployer);
    Logger.recordMetadata("Bot", String.valueOf(WhoAmI.bot));
    Logger.recordMetadata("Appendages", Arrays.toString(WhoAmI.appendages));
    Logger.recordMetadata("SN", HALUtil.getSerialNumber());

    switch (WhoAmI.mode) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter("/U"));
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case SIM:
        Logger.addDataReceiver(new WPILOGWriter("SimLogs/"));
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case REPLAY:
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(
            new WPILOGWriter(
                LogFileUtil.addPathSuffix(
                    logPath,
                    "_replay" + new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date()))));
        break;
    }
    Logger.start();
    CustomAlerts.makeCANFailAlerts(0.9);
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    PeriodicRunnable.runPeriodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {
    SwerveSimManager.getInstance().propagate();
  }
}
