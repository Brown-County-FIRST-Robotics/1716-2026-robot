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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.subsystems.drive.Drive.HoldMode;
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
import frc.robot.utils.buttonbox.ButtonBox;
import frc.robot.utils.buttonbox.ControlPanel;
import gg.questnav.questnav.QuestNav;
import java.io.IOException;
import java.util.Set;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

class MirroredAutoInfo {
  public String name;
  public String desc;
  public Supplier<Command> parallelCommand;
  boolean shakeOnEnd;

  public MirroredAutoInfo(
      String name, String desc, Supplier<Command> parallelCommand, boolean shakeOnEnd) {
    this.name = name;
    this.desc = desc;
    this.parallelCommand = parallelCommand;
    this.shakeOnEnd = shakeOnEnd;
  }

  public static Command generateParallelCommand(
      Drive drive,
      Intake intake,
      Shooter shooter,
      Rollers rollers,
      double intakeWaitSeconds,
      double shootWaitSeconds) {
    return Commands.race(
        Commands.waitSeconds(intakeWaitSeconds)
            .andThen(
                intake
                    .extendHopper()
                    .andThen(intake.intake().withTimeout(shootWaitSeconds - intakeWaitSeconds))
                    .andThen(
                        Commands.runOnce(
                                () -> {
                                  shooter.setShooterSpeed(80);
                                  shooter.quickServoCommand(1);
                                })
                            .andThen(
                                Commands.waitSeconds(0.4)
                                    .andThen(
                                        Commands.run(() -> rollers.setSpeeds(20, 20, 20), rollers)))
                            .alongWith(
                                Commands.run(
                                    () -> shooter.trackBothToShoot(drive.getPose()), shooter))
                            .alongWith(intake.shake()))),
        Commands.waitSeconds(14.5));
  }
}

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer extends PeriodicRunnable {
  // Subsystems
  private final Drive drive;
  private final Quest quest;
  private Shooter shooter;
  private Rollers rollers;
  private Intake intake;
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController opcon = new CommandXboxController(1);
  private ButtonBox buttonBox = new ButtonBox(2);
  private ControlPanel controlPanel = new ControlPanel(buttonBox);
  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<Pose2d> initPosChooser;

  private final UsbCamera camera;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    super();

    camera = CameraServer.startAutomaticCapture();
    camera.setResolution(320, 200);
    camera.setFPS(30);
    quest = new Quest(new QuestIOQuest(new QuestNav()));
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                quest,
                new GyroIO() {
                  @Override
                  public void updateInputs(GyroIOInputs inputs) {
                    inputs.connected = quest.trust();
                    inputs.yawPosition = quest.gyroLikeYaw();
                  }
                },
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        shooter = new Shooter(new ShooterIOKrakens(62, 9), new TurretIOKrakens(36, 35, 22));
        rollers = new Rollers(new RollersIOKraken(43, 34, 37));
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
                quest,
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
                quest,
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
        "Centered on Hub, intake toward hub",
        new Pose2d(3.638606071472168, 4.050412178039551, Rotation2d.kZero));
    initPosChooser.addOption(
        "Centered on Hub intake toward drivers",
        new Pose2d(3.638606071472168, 4.050412178039551, Rotation2d.k180deg));

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
      // side is the same as the human player. In Choreo as of 3/30/26, that is
      // the bottom left corner.
      MirroredAutoInfo[] items = {
        // (name, description, parallel command - shooting, intake, etc)
        new MirroredAutoInfo(
            "FuelToucher",
            "FULL FIELD - Push balls to side -> end on same trench",
            () -> Commands.none(),
            false),
        new MirroredAutoInfo(
            "FuelCollectorHalf",
            "HALF FIELD - Push balls to side -> end on same trench",
            () -> Commands.none(),
            false),
        new MirroredAutoInfo(
            "FuelCollectorHalf",
            "HALF FIELD - ADVANCED - End on same trench",
            () -> MirroredAutoInfo.generateParallelCommand(drive, intake, shooter, rollers, 0.2, 9),
            true)
        // Add more here here
      };

      for (MirroredAutoInfo item : items) {
        String name = item.name;
        String desc = item.desc;
        Supplier<Command> parallel = item.parallelCommand;
        boolean shake = item.shakeOnEnd;

        PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(name);
        autoChooser.addOption(
            "Choreo - Human player side - " + desc,
            parallel
                .get()
                .alongWith(
                    AutoBuilder.followPath(path)
                        .andThen(shake ? DriveCommands.shake(drive) : Commands.none())));
        autoChooser.addOption(
            "Choreo - Depot side - " + desc,
            parallel
                .get()
                .alongWith(
                    AutoBuilder.followPath(path.mirrorPath())
                        .andThen(shake ? DriveCommands.shake(drive) : Commands.none())));
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
            false,
            () -> -controller.getLeftX() / (controller.leftTrigger().getAsBoolean() ? 2 : 1),
            false,
            () -> -controller.getRightX() / (controller.leftTrigger().getAsBoolean() ? 2 : 1),
            false,
            () -> {
              return controlPanel.questDown().getAsBoolean()
                  || controller.rightStick().getAsBoolean();
            },
            () -> false));

    controller
        .leftBumper()
        .whileTrue(
            Commands.defer(
                () ->
                    DriveCommands.joystickDrive(
                        drive,
                        () ->
                            -controller.getLeftY()
                                / (controller.leftTrigger().getAsBoolean() ? 2 : 1),
                        false,
                        () ->
                            DriverStation.getAlliance().isPresent()
                                    && DriverStation.getAlliance().get() == Alliance.Red
                                ? (0.639445 - 0.0254)
                                : 8.069275 - (0.639445 - 0.0254),
                        true,
                        () -> {
                          double r = drive.getRotation().getCos();
                          return r > 0 ? 0 : Math.PI;
                        },
                        true,
                        () -> {
                          return opcon.a().getAsBoolean() || controller.rightStick().getAsBoolean();
                        },
                        () -> false),
                Set.of(drive)));
    controller
        .rightBumper()
        .whileTrue(
            Commands.defer(
                () ->
                    DriveCommands.joystickDrive(
                        drive,
                        () ->
                            -controller.getLeftY()
                                / (controller.leftTrigger().getAsBoolean() ? 2 : 1),
                        false,
                        () ->
                            DriverStation.getAlliance().isPresent()
                                    && DriverStation.getAlliance().get() == Alliance.Red
                                ? 8.069275 - (0.639445 - 0.0254)
                                : (0.639445 - 0.0254),
                        true,
                        () -> {
                          double r = drive.getRotation().getCos();
                          return r > 0 ? 0 : Math.PI;
                        },
                        true,
                        () -> {
                          return opcon.a().getAsBoolean() || controller.rightStick().getAsBoolean();
                        },
                        () -> false),
                Set.of(drive)));

    controlPanel
        .questDown()
        .whileTrue(Commands.run(() -> shooter.commandTurret(Rotation2d.k180deg), shooter));

    // Track by default
    shooter.setDefaultCommand(
        Commands.run(
            () -> {
              shooter.commandTurretToTrack(
                  drive
                      .getPose()
                      .plus(
                          new Transform2d(
                              0,
                              0,
                              Rotation2d.fromRadians(
                                  drive.getChassisSpeeds().omegaRadiansPerSecond
                                      * OurConstants.AIM_LOOKAHEAD))));
              shooter.quickServoCommand(0);
            },
            shooter));

    // Switch to X pattern when button is pressed
    // controller.leftBumper().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Stop intake when no commands are running
    intake.setDefaultCommand(intake.intakeStop());

    // Reset initial pos on auto init
    RobotModeTriggers.autonomous()
        .onTrue(Commands.runOnce(() -> drive.setPose(FieldConstants.flip(initPosChooser.get()))));

    RobotModeTriggers.disabled().onFalse(Commands.runOnce(drive::resetHold));

    new Trigger(intake::isExtenderConnected).onTrue(Commands.runOnce(intake::setZeroPosition));
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(intake::setZeroPosition));

    new Trigger(quest::trust)
        .onFalse(
            Commands.runEnd(
                () -> controller.setRumble(RumbleType.kLeftRumble, 1),
                () -> controller.setRumble(RumbleType.kBothRumble, 0)));
    // Press right trigger to run shooter startup
    controller
        .rightTrigger(0.7)
        .whileTrue(
            Commands.runOnce(() -> shooter.setShooterSpeed(80))
                .alongWith(DriveCommands.shake(drive))
                .alongWith(intake.shake())
                .alongWith(
                    Commands.waitSeconds(0.4)
                        .andThen(Commands.run(() -> rollers.setSpeeds(40, 20, 20), rollers)))
                .alongWith(
                    Commands.race(
                            Commands.run(() -> controller.setRumble(RumbleType.kRightRumble, 0.5)),
                            Commands.waitSeconds(0.375))
                        .andThen(
                            Commands.run(() -> controller.setRumble(RumbleType.kBothRumble, 1.0))))
                .alongWith(Commands.run(() -> shooter.trackBothToShoot(drive.getPose()), shooter))
                .finallyDo(
                    () -> {
                      shooter.quickWheelCommand(0);
                      rollers.setSpeeds(0, 0, 0);
                      shooter.quickServoCommand(0);
                      controller.setRumble(RumbleType.kBothRumble, 0);
                    }));

    // Hood positions
    // controller.y().whileTrue(Commands.run(() -> shooter.quickServoCommand(0), shooter));
    // controller.b().whileTrue(Commands.run(() -> shooter.quickServoCommand(1), shooter));
    controlPanel
        .hoodSafePosition()
        .whileTrue(Commands.run(() -> shooter.quickServoCommand(0), shooter));
    controlPanel
        .hoodShootPosition()
        .whileTrue(Commands.run(() -> shooter.quickServoCommand(1), shooter));

    // Intake/hopper control
    controlPanel.hopperOut().whileTrue(intake.extendHopperVelocity(3));
    controlPanel.hopperIn().whileTrue(intake.retractHopperVelocity(15));
    // opcon.rightTrigger().onTrue(intake.extendHopper());
    // opcon.leftTrigger().onTrue(intake.retractHopper());
    controlPanel.intakeForward().whileTrue(intake.intake());
    controlPanel
        .intakeReverse()
        .whileTrue(
            intake
                .intakeReverse()
                .alongWith(
                    Commands.runEnd(
                        () -> rollers.setSpeeds(20, -20, 0),
                        () -> rollers.setSpeeds(0, 0, 0),
                        rollers)));

    controller.povUp().whileTrue(intake.extendHopperVelocity(3));
    controller.povDown().whileTrue(intake.retractHopperVelocity(15));
    controller.povRight().onTrue(intake.intake());
    controller
        .povLeft()
        .whileTrue(
            intake
                .intakeReverse()
                .alongWith(
                    Commands.runEnd(
                        () -> rollers.setSpeeds(20, -20, 0),
                        () -> rollers.setSpeeds(0, 0, 0),
                        rollers)));

    // opcon.povUp().whileTrue(intake.extendHopperVelocity());
    // opcon.povDown().whileTrue(intake.retractHopperVelocity());
    // opcon.rightTrigger().onTrue(intake.extendHopper());
    // opcon.leftTrigger().onTrue(intake.retractHopper());
    // opcon.rightBumper().whileTrue(intake.intake());
    // opcon
    //     .leftBumper()
    //     .whileTrue(
    //         intake
    //             .intakeReverse()
    //             .alongWith(
    //                 Commands.runEnd(
    //                     () -> rollers.setSpeeds(20, -20, 0),
    //                     () -> rollers.setSpeeds(0, 0, 0),
    //                     rollers)));

    // Rotation snapping
    controller
        .y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  drive.targetAngle =
                      DriverStation.getAlliance().isPresent()
                              && DriverStation.getAlliance().get() == Alliance.Blue
                          ? Rotation2d.kZero
                          : Rotation2d.k180deg;
                  drive.holdingAngle = HoldMode.HOLD;
                }));
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                () -> {
                  drive.targetAngle =
                      DriverStation.getAlliance().isPresent()
                              && DriverStation.getAlliance().get() == Alliance.Blue
                          ? Rotation2d.kCW_90deg
                          : Rotation2d.kCCW_90deg;
                  drive.holdingAngle = HoldMode.HOLD;
                }));
    controller
        .a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  drive.targetAngle =
                      DriverStation.getAlliance().isPresent()
                              && DriverStation.getAlliance().get() == Alliance.Blue
                          ? Rotation2d.k180deg
                          : Rotation2d.kZero;
                  drive.holdingAngle = HoldMode.HOLD;
                }));
    controller
        .x()
        .onTrue(
            Commands.runOnce(
                () -> {
                  drive.targetAngle =
                      DriverStation.getAlliance().isPresent()
                              && DriverStation.getAlliance().get() == Alliance.Blue
                          ? Rotation2d.kCCW_90deg
                          : Rotation2d.kCW_90deg;
                  drive.holdingAngle = HoldMode.HOLD;
                }));

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
      matchTime = -1;
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
