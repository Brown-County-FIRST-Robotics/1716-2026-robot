// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOKraken;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOKraken;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOKrakens;
import frc.robot.subsystems.shooter.TurretIO;
import frc.robot.subsystems.shooter.TurretIOKrakens;
import frc.robot.subsystems.vision.Quest;
import frc.robot.subsystems.vision.QuestIOQuest;
import frc.robot.utils.PeriodicRunnable;
import gg.questnav.questnav.QuestNav;
import java.io.IOException;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer extends PeriodicRunnable {
  // Subsystems
  private final Drive drive;
  private final Quest qwest;
  private Shooter shooter;
  private Rollers rollers;
  private Intake intake;
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController opcon = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<Pose2d> initPosChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    super();
    Logger.recordOutput("Time Left", -1);
    qwest = new Quest(new QuestIOQuest(new QuestNav()));
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                qwest,
                new GyroIO() {
                  @Override
                  public void updateInputs(GyroIOInputs inputs) {
                    inputs.connected = qwest.isConnected();
                    inputs.yawPosition = qwest.gyroLikeYaw();
                  }
                },
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        shooter = new Shooter(new ShooterIOKrakens(62, 9), new TurretIOKrakens(36, 35, 22));
        rollers = new Rollers(new RollersIOKraken(43, 37));
        intake = new Intake(new IntakeIOKraken(40, 38));

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                qwest,
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        shooter = new Shooter(new ShooterIO() {}, new TurretIO() {});
        intake = new Intake(new IntakeIO() {});
        rollers = new Rollers(new RollersIO() {});

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                qwest,
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

    initPosChooser = new LoggedDashboardChooser<Pose2d>("Starting Positions");
    initPosChooser.addDefaultOption("CHANGE ME", new Pose2d());
    initPosChooser.addOption(
        "Blue - Human player - Trench",
        new Pose2d(3.638606071472168, 1.1230977773666382, Rotation2d.kZero));
    initPosChooser.addOption(
        "Blue - Depot - Trench",
        new Pose2d(3.638606071472168, 8.0692 - 1.1230977773666382, Rotation2d.kZero));
    initPosChooser.addOption(
        "Red - Human player - Trench",
        new Pose2d(3.638606071472168, 8.0692 - 1.1230977773666382, Rotation2d.kZero));
    initPosChooser.addOption(
        "Red - Depot - Trench",
        new Pose2d(3.638606071472168, 1.1230977773666382, Rotation2d.kZero));
    initPosChooser.addOption(
        "Centered on Hub", new Pose2d(3.638606071472168, 4.050412178039551, Rotation2d.kZero));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addDefaultOption("None", Commands.none());
    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // ########## Choreo autos ##########
    try {
      // Single-side autos that don't need to be mirrored
      autoChooser.addOption(
          "Choreo - Middle -> climb",
          AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("MidToClimb")));

      // Both-side autos

      // ### NOTE ###
      // It is important that autos are developed as *BLUE* and the originating
      // side is the same as the human player. In Choreo as of 2/20/26, that is
      // the bottom left corner.
      String[][] items = {
        // {name, description}
        {"FuelToucher", "FULL FIELD - Push balls to side -> end on same trench"},
        {"FuelCollectorFull", "FULL FIELD - Push balls to side -> end on opposite trench"},
        {"FuelCollectorHalf", "HALF FIELD - Push balls to side -> end on same trench"}
        // Add more here here
      };

      for (String[] item : items) {
        String name = item[0];
        String desc = item[1];

        PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(name);
        autoChooser.addOption("Choreo - Human player side - " + desc, AutoBuilder.followPath(path));
        autoChooser.addOption(
            "Choreo - Depot side - " + desc, AutoBuilder.followPath(path.mirrorPath()));
      }
    } catch (FileVersionException | IOException | ParseException e) {
      e.printStackTrace();
    }

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    // with 4x slowmode on op's right trigger
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY() / (controller.leftTrigger().getAsBoolean() ? 2 : 1),
            () -> -controller.getLeftX() / (controller.leftTrigger().getAsBoolean() ? 2 : 1),
            () -> -controller.getRightX() / (controller.leftTrigger().getAsBoolean() ? 2 : 1),
            opcon.a(),
            controller.a()));

    controller
        .leftStick()
        .whileTrue(
            DriveCommands.driveToPose(
                drive, new Pose2d(drive.getPose().getX(), 0.6238498687744141, Rotation2d.kZero)));

    opcon.a().whileTrue(Commands.run(() -> shooter.commandTurret(Rotation2d.k180deg)));

    // Track by default
    shooter.setDefaultCommand(
        Commands.run(() -> shooter.commandTurretToTrack(drive.getPose()), shooter));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset initial pos on auto init
    RobotModeTriggers.autonomous()
        .onTrue(Commands.runOnce(() -> drive.setPose(FieldConstants.flip(initPosChooser.get()))));

    new Trigger(intake::isExtenderConnected).onTrue(Commands.runOnce(intake::setZeroPosition));
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(intake::setZeroPosition));
    // Press right trigger to run shooter startup
    controller
        .rightTrigger(0.7)
        .whileTrue(
            Commands.run(
                    () -> {
                      shooter.setShooterSpeed(80);
                      shooter.quickServoCommand(1);
                    },
                    shooter)
                .alongWith(
                    Commands.waitSeconds(0.4)
                        .andThen(Commands.run(() -> rollers.setSpeeds(20, 20), rollers)))
                .alongWith(
                    Commands.race(
                            Commands.run(() -> controller.setRumble(RumbleType.kRightRumble, 0.5)),
                            Commands.waitSeconds(0.375))
                        .andThen(
                            Commands.run(() -> controller.setRumble(RumbleType.kBothRumble, 1.0))))
                .finallyDo(
                    () -> {
                      shooter.quickWheelCommand(0);
                      rollers.setSpeeds(0, 0);
                      shooter.quickServoCommand(0);
                      controller.setRumble(RumbleType.kBothRumble, 0);
                    }));

    opcon
        .povLeft()
        .onTrue(
            Commands.runOnce(
                () ->
                    shooter.commandTurret(
                        shooter.getTurretRotation().plus(Rotation2d.fromRotations(0.1)))));
    opcon
        .povRight()
        .onTrue(
            Commands.runOnce(
                () ->
                    shooter.commandTurret(
                        shooter.getTurretRotation().plus(Rotation2d.fromRotations(-0.1)))));

    // Hood positions
    // controller.y().whileTrue(Commands.run(() -> shooter.quickServoCommand(0), shooter));
    // controller.b().whileTrue(Commands.run(() -> shooter.quickServoCommand(1), shooter));
    opcon.y().whileTrue(Commands.run(() -> shooter.quickServoCommand(0), shooter));
    opcon.b().whileTrue(Commands.run(() -> shooter.quickServoCommand(1), shooter));

    // Intake/hopper control
    opcon.povUp().whileTrue(intake.extendHopperVelocity());
    opcon.povDown().whileTrue(intake.retractHopperVelocity());
    opcon.rightTrigger().onTrue(intake.extendHopper());
    opcon.leftTrigger().onTrue(intake.retractHopper());
    opcon.rightBumper().whileTrue(intake.intake());
    opcon
        .leftBumper()
        .whileTrue(
            intake
                .intakeReverse()
                .alongWith(
                    Commands.runEnd(
                        () -> rollers.setSpeeds(-20, 0), () -> rollers.setSpeeds(0, 0))));

    // Dpad
    controller
        .povUp()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY() / (controller.leftTrigger().getAsBoolean() ? 2 : 1),
                () -> -controller.getLeftX() / (controller.leftTrigger().getAsBoolean() ? 2 : 1),
                () -> Rotation2d.kZero));
    controller
        .povRight()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY() / (controller.leftTrigger().getAsBoolean() ? 2 : 1),
                () -> -controller.getLeftX() / (controller.leftTrigger().getAsBoolean() ? 2 : 1),
                () -> Rotation2d.kCW_90deg));
    controller
        .povDown()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY() / (controller.leftTrigger().getAsBoolean() ? 2 : 1),
                () -> -controller.getLeftX() / (controller.leftTrigger().getAsBoolean() ? 2 : 1),
                () -> Rotation2d.k180deg));
    controller
        .povLeft()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY() / (controller.leftTrigger().getAsBoolean() ? 2 : 1),
                () -> -controller.getLeftX() / (controller.leftTrigger().getAsBoolean() ? 2 : 1),
                () -> Rotation2d.kCCW_90deg));
    shooter.register();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  @Override
  public void periodic() {
    double matchTime = DriverStation.getMatchTime();

    // Shift boundaries (seconds remaining in teleop)
    // Teleop = 140s total
    // Transition: 140-130, Shift1: 130-105, Shift2: 105-80,
    // Shift3: 80-55, Shift4: 55-30, EndGame: 30-0
    double timeUntilShiftEnd;
    String currentPhase;

    if (DriverStation.isDisabled()) {
      currentPhase = "Disabled";
      timeUntilShiftEnd = -1;
    } else if (DriverStation.isAutonomous()) {
      currentPhase = "Auto";
      timeUntilShiftEnd = matchTime;
    } else if (matchTime > 130) {
      currentPhase = "Transition";
      timeUntilShiftEnd = matchTime - 130.0;
    } else if (matchTime > 105) {
      currentPhase = "Shift 1";
      timeUntilShiftEnd = matchTime - 105.0;
    } else if (matchTime > 80) {
      currentPhase = "Shift 2";
      timeUntilShiftEnd = matchTime - 80.0;
    } else if (matchTime > 55) {
      currentPhase = "Shift 3";
      timeUntilShiftEnd = matchTime - 55.0;
    } else if (matchTime > 30) {
      currentPhase = "Shift 4";
      timeUntilShiftEnd = matchTime - 30.0;
    } else {
      currentPhase = "End Game";
      timeUntilShiftEnd = matchTime;
    }

    Logger.recordOutput("Time Left", matchTime);
    Logger.recordOutput("Shift Time Left", Math.round(timeUntilShiftEnd * 100) / 100.0);
    Logger.recordOutput("Current Phase", currentPhase);
  }
}
