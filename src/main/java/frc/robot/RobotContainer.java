// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.IMUIO;
import frc.robot.subsystems.IMUIONavx;
import frc.robot.subsystems.IMUIOSim;
import frc.robot.subsystems.swerve.Module;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOSim;
import frc.robot.subsystems.swerve.ModuleIOSparkFX;
import frc.robot.subsystems.swerve.SwerveDrivetrain;
import frc.robot.subsystems.vision.*;
import frc.robot.utils.buttonbox.ButtonBox;
import frc.robot.utils.buttonbox.ManipulatorPanel;
import frc.robot.utils.buttonbox.OverridePanel;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final ButtonBox buttonBox = new ButtonBox(2);
  private final ManipulatorPanel manipulatorPanel = new ManipulatorPanel(buttonBox);
  private final OverridePanel overridePanel = new OverridePanel(buttonBox);
  private final SwerveDrivetrain driveSys;
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    autoChooser = new LoggedDashboardChooser<>("Auto chooser");
    if (WhoAmI.mode != WhoAmI.Mode.REPLAY) {
      switch (WhoAmI.bot) {
        case SIMSWERVEBASE:
          driveSys =
              new SwerveDrivetrain(
                  new Module(new ModuleIOSim(0), 0),
                  new Module(new ModuleIOSim(1), 1),
                  new Module(new ModuleIOSim(2), 2),
                  new Module(new ModuleIOSim(3), 3),
                  new IMUIOSim());
          break;
        case SWERVEBASE:
          driveSys =
              new SwerveDrivetrain(
                  new Module(new ModuleIOSparkFX(24, 29, "FL"), 0),
                  new Module(new ModuleIOSparkFX(23, 19, "FR"), 1),
                  new Module(new ModuleIOSparkFX(20, 40, "BL"), 2),
                  new Module(new ModuleIOSparkFX(22, 9, "BR"), 3),
                  new IMUIONavx());
          var vision =
              new FusedVision(
                  driveSys,
                  new Transform3d(
                      new Translation3d(-0.15, -0.17, 0),
                      new Rotation3d(00.0 * Math.PI / 180.0, 0, 180.0 * Math.PI / 180.0)),
                  new VisionSLAMIOQuest(),
                  new VisionIOPhotonVision(
                      "TH_CAM0",
                      new Transform3d(
                          new Translation3d(-12 * 0.0254, -9.5 * 0.0254, 0),
                          new Rotation3d(-4.0 * Math.PI / 180.0, 0, Math.PI))));

          break;
        default:
          driveSys =
              new SwerveDrivetrain(
                  new Module(new ModuleIOSim(0), 0),
                  new Module(new ModuleIOSim(1), 1),
                  new Module(new ModuleIOSim(2), 2),
                  new Module(new ModuleIOSim(3), 3),
                  new IMUIOSim());
      }
      for (var appendage : WhoAmI.appendages) {}

    } else {
      switch (WhoAmI.bot) {
        case SIMSWERVEBASE:
          driveSys =
              new SwerveDrivetrain(
                  new Module(new ModuleIO() {}, 0),
                  new Module(new ModuleIO() {}, 1),
                  new Module(new ModuleIO() {}, 2),
                  new Module(new ModuleIO() {}, 3),
                  new IMUIO() {});
          break;
        case SWERVEBASE:
          driveSys =
              new SwerveDrivetrain(
                  new Module(new ModuleIO() {}, 0),
                  new Module(new ModuleIO() {}, 1),
                  new Module(new ModuleIO() {}, 2),
                  new Module(new ModuleIO() {}, 3),
                  new IMUIO() {});
          // TODO: this is a bad way of doing this
          var vision =
              new FusedVision(
                  driveSys,
                  new Transform3d(
                      new Translation3d(),
                      new Rotation3d(0.0 * Math.PI / 180.0, 0, 90.0 * Math.PI / 180.0)),
                  new VisionSLAMIO() {},
                  new VisionIO() {});
          break;
        default:
          driveSys =
              new SwerveDrivetrain(
                  new Module(new ModuleIO() {}, 0),
                  new Module(new ModuleIO() {}, 1),
                  new Module(new ModuleIO() {}, 2),
                  new Module(new ModuleIO() {}, 3),
                  new IMUIO() {});
      }
    }

    TeleopDrive teleopDrive = configureSharedBindings();

    AutoFactory autoFactory =
        new AutoFactory(
            driveSys::getPosition,
            driveSys::setPosition, // TODO: don't do this (ie: give fake function)
            driveSys::followTrajectory,
            false,
            driveSys);
    autoChooser.addDefaultOption("Nothing", Commands.none());
    autoChooser.addOption(
        "Move backward 3s",
        Commands.runEnd(
                () -> driveSys.humanDrive(new ChassisSpeeds(-1, 0, 0)),
                () -> driveSys.humanDrive(new ChassisSpeeds()),
                driveSys)
            .raceWith(Commands.waitSeconds(3)));

    // Level represents the height of the elevator preset
    for (int level = 1; level <= 3; level++) {
      // ************ SCORE 1 CORAL AND RETURN TO STATION ************
      // Routines for all 3 starting positions
      AutoRoutine lAuto = autoFactory.newRoutine("L-Auto");
      AutoRoutine mAuto = autoFactory.newRoutine("M-Auto");
      AutoRoutine rAuto = autoFactory.newRoutine("R-Auto");

      // Add all of the trajectories
      AutoTrajectory lAlign = lAuto.trajectory("L-Auto", 0);
      AutoTrajectory lPickup = lAuto.trajectory("L-Auto", 1);

      AutoTrajectory mAlign = mAuto.trajectory("M-Auto", 0);
      AutoTrajectory mPickup = mAuto.trajectory("M-Auto", 1);

      AutoTrajectory rAlign = rAuto.trajectory("R-Auto", 0);
      AutoTrajectory rPickup = rAuto.trajectory("R-Auto", 1);

      AutoRoutine temp = autoFactory.newRoutine("test");
      AutoTrajectory tempTraj = temp.trajectory("R-Auto", 0);
      temp.active().onTrue(tempTraj.cmd());
      autoChooser.addOption("Test", temp.cmd());

      // This is the command to drop the current coral

      // This configures the actual commands that end up being run. It uses trajectories as well as
      // other cmds

      // Add paths to the auto chooser
      autoChooser.addOption("Left 1 Coral Lvl " + level + " - Choreo", lAuto.cmd());
      autoChooser.addOption("Middle 1 Coral Lvl " + level + " - Choreo", mAuto.cmd());
      autoChooser.addOption("Right 1 Coral Lvl " + level + " - Choreo", rAuto.cmd());
    }

    // ************ DRIVE TO CORAL STATION ************
    // Make the routines
    // They will drive to the nearest station, middle has an auto to go to either station
    // It reuses the first segment of the main autos
    AutoRoutine lLineup = autoFactory.newRoutine("L-Lineup");
    AutoRoutine mLineup = autoFactory.newRoutine("M-Lineup");
    AutoRoutine rLineup = autoFactory.newRoutine("R-Lineup");

    // Load all the trajectories
    AutoTrajectory lAlign = lLineup.trajectory("L-Auto", 0);
    AutoTrajectory mAlign = mLineup.trajectory("M-Auto", 0);
    AutoTrajectory rAlign = rLineup.trajectory("R-Auto", 0);

    // Merge all the commands into the auto routines
    lLineup.active().onTrue(lAlign.cmd());
    mLineup.active().onTrue(mAlign.cmd());
    rLineup.active().onTrue(rAlign.cmd());

    // Add the new paths to the auto chooser
    autoChooser.addOption("Left lineup - Choreo", lLineup.cmd());
    autoChooser.addOption("Middle lineup - Choreo", mLineup.cmd());
    autoChooser.addOption("Right lineup - Choreo", rLineup.cmd());

    if (WhoAmI.isDemoMode) {
      configureDemoBindings(teleopDrive);
    } else {
      configureCompBindings();
    }
  }

  public void configureAutos() {}

  /** Updates the pose estimator to use the correct initial pose */
  public void setPose(Pose2d pose) {
    driveSys.setPosition(pose);
  }

  private void configureDemoBindings(TeleopDrive teleopDrive) {
    teleopDrive.isKidMode = false;
  }

  private void configureCompBindings() {
    // Manipulator Presets

    driverController.back().onTrue(Commands.runOnce(() -> driveSys.setPosition(Pose2d.kZero)));
  }

  private TeleopDrive configureSharedBindings() {
    var teleopDrive = new TeleopDrive(driveSys, driverController, overridePanel);

    driveSys.setDefaultCommand(teleopDrive);

    return teleopDrive;
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
