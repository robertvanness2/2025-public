// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.quixlib.advantagekit.LoggerHelper;
import frc.quixlib.devices.QuixCANBus;
import frc.quixlib.devices.QuixPigeon2;
import frc.quixlib.vision.Fiducial;
import frc.quixlib.vision.PhotonVisionCamera;
import frc.quixlib.vision.QuixVisionCamera;
import frc.quixlib.vision.QuixVisionSim;
import frc.robot.AlignmentState.ReefLevel;
import frc.robot.AlignmentState.ReefStackChoice;
import frc.robot.ScoringKinematics.ScoringKinematicsOutput;
import frc.robot.commands.AlgaeGround;
import frc.robot.commands.EmergencyUntangle;
import frc.robot.commands.IntakeOrScoreAlgaeCommand;
import frc.robot.commands.RaiseBeforeStow;
import frc.robot.commands.RunFunnelWhenReady;
import frc.robot.commands.SafelyStowAndIntakeCommand;
import frc.robot.commands.ScoreCoralCommand;
import frc.robot.commands.TeleopSwerveCommand;
import frc.robot.commands.autos.AutoCenter;
import frc.robot.commands.autos.AutoCommand;
import frc.robot.commands.autos.AutoCrazyTest;
import frc.robot.commands.autos.AutoFourPieceLeft;
import frc.robot.commands.autos.AutoFourPieceLeftEndFront;
import frc.robot.commands.autos.AutoFourPieceRight;
import frc.robot.commands.autos.AutoFourPieceRightEndFront;
import frc.robot.commands.autos.AutoSimpleTest;
import frc.robot.commands.autos.AutoSquareTest;
import frc.robot.commands.autos.FrontOnlyAutoLeftStart;
import frc.robot.commands.autos.FrontOnlyAutoRightStart;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.ArrayList;
import java.util.Arrays;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // Controllers
  public final XboxController driverXbox = new XboxController(0);

  // ABXY Buttons
  private final JoystickButton buttonA =
      new JoystickButton(driverXbox, XboxController.Button.kA.value);
  private final JoystickButton buttonB =
      new JoystickButton(driverXbox, XboxController.Button.kB.value);
  private final JoystickButton buttonX =
      new JoystickButton(driverXbox, XboxController.Button.kX.value);
  private final JoystickButton buttonY =
      new JoystickButton(driverXbox, XboxController.Button.kY.value);

  // Bumpers
  private final JoystickButton bumperLeft =
      new JoystickButton(driverXbox, XboxController.Button.kLeftBumper.value);
  private final JoystickButton bumperRight =
      new JoystickButton(driverXbox, XboxController.Button.kRightBumper.value);

  // Triggers
  private final Trigger leftTrigger = new Trigger(() -> driverXbox.getLeftTriggerAxis() > 0.2);
  private final Trigger rightTrigger = new Trigger(() -> driverXbox.getRightTriggerAxis() > 0.2);

  // D-pad Buttons
  private final Trigger dPadUp = new Trigger(() -> driverXbox.getPOV() == 0);
  private final Trigger dPadRight = new Trigger(() -> driverXbox.getPOV() == 90);
  private final Trigger dPadDown = new Trigger(() -> driverXbox.getPOV() == 180);
  private final Trigger dPadLeft = new Trigger(() -> driverXbox.getPOV() == 270);

  // Menu Buttons
  private final JoystickButton buttonStart =
      new JoystickButton(driverXbox, XboxController.Button.kStart.value);
  private final JoystickButton buttonBack =
      new JoystickButton(driverXbox, XboxController.Button.kBack.value);

  // Paddles
  // Bind sticks to paddles on controller
  private final Trigger leftPaddle =
      new JoystickButton(driverXbox, XboxController.Button.kLeftStick.value);
  private final Trigger rightPaddle =
      new JoystickButton(driverXbox, XboxController.Button.kRightStick.value);

  // CANbuses
  private final QuixCANBus rioBus = new QuixCANBus();
  private final QuixCANBus canivoreBus = new QuixCANBus(Constants.kCanivoreName);

  // Sensors
  private final QuixPigeon2 imu =
      new QuixPigeon2(
          Constants.IMU.pigeonID,
          QuixPigeon2.makeDefaultConfig().setGyroTrimZ(Constants.IMU.gyroTrimZ));
  private final ArrayList<QuixVisionCamera> cameras =
      new ArrayList<>(
          Arrays.asList(
              new PhotonVisionCamera(
                  "frontleft",
                  Constants.Cameras.LeftCam.robotToCameraT,
                  Constants.Cameras.LeftCam.pipelineConfigs),
              new PhotonVisionCamera(
                  "frontright",
                  Constants.Cameras.RightCam.robotToCameraT,
                  Constants.Cameras.RightCam.pipelineConfigs)));
  private final ArrayList<QuixVisionCamera> localizationCameras =
      new ArrayList<>(Arrays.asList(cameras.get(0), cameras.get(1)));

  // Simulation & viz
  private final QuixVisionSim visionSim = new QuixVisionSim(cameras, Fiducials.aprilTagFiducials);
  private final Field2d fieldViz = visionSim.getSimField();

  // Subsystems
  private final SwerveSubsystem swerve =
      new SwerveSubsystem(imu, localizationCameras, visionSim, fieldViz);
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final GripperSubsystem gripper = new GripperSubsystem();
  private final FunnelSubsystem funnel = new FunnelSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();

  // Misc.
  private final AlignmentState aState = AlignmentState.getInstance();
  private final LoggedDashboardChooser<AutoCommand> autoChooser =
      new LoggedDashboardChooser<>("Auto Chooser");
  private AutoCommand lastSelectedAuto = null;

  public RobotContainer() {
    // Autos
    autoChooser.addDefaultOption(
        "Four Piece Left", new AutoFourPieceLeft(gripper, elevator, arm, swerve));
    autoChooser.addOption(
        "Four Piece Right", new AutoFourPieceRight(gripper, elevator, arm, swerve));
    autoChooser.addOption("Center Back", new AutoCenter(gripper, elevator, arm, swerve));
    autoChooser.addOption("Front Left", new FrontOnlyAutoLeftStart(gripper, elevator, arm, swerve));
    autoChooser.addOption(
        "Front Right", new FrontOnlyAutoRightStart(gripper, elevator, arm, swerve));
    autoChooser.addOption(
        "Four Piece Right END FRONT",
        new AutoFourPieceRightEndFront(gripper, elevator, arm, swerve));
    autoChooser.addOption(
        "Four Piece Left END FRONT", new AutoFourPieceLeftEndFront(gripper, elevator, arm, swerve));

    // Testing Autos
    autoChooser.addOption("Simple Test", new AutoSimpleTest(swerve));
    autoChooser.addOption("Square Test", new AutoSquareTest(swerve));
    autoChooser.addOption("Crazy Test", new AutoCrazyTest(swerve));

    // Default commands
    swerve.setDefaultCommand(new TeleopSwerveCommand(swerve, driverXbox));
    elevator.setDefaultCommand(
        new SafelyStowAndIntakeCommand(elevator, arm, gripper, () -> swerve.getSingleTagPose()));
    funnel.setDefaultCommand(
        new RunFunnelWhenReady(
            funnel,
            () -> {
              return elevator.isElevatorAtHeight(
                      Constants.Elevator.stowHeight, Units.inchesToMeters(1.0))
                  && arm.isArmAtAngle(Constants.Arm.stowAngle, Math.toRadians(3.0))
                  && gripper.isWristAtAngle(Constants.Gripper.stowAngle, Math.toRadians(3.0))
                  && !gripper.hasPiece()
                  && !gripper.hasAlgae
                  && !climber.isCliming;
            }));

    configureBindings();

    // Setup simulation
    if (Robot.isSimulation()) {
      SimulatedArena.getInstance().resetFieldForAuto();
    }
  }

  private void configureBindings() {
    // Gyro reset
    buttonStart.onTrue(
        new InstantCommand(
            () -> {
              final var alliance = DriverStation.getAlliance();
              swerve.setContinuousYaw(
                  alliance.isPresent() && alliance.get() == Alliance.Blue ? 0.0 : Math.PI);
            }));
    leftPaddle.whileTrue(new AlgaeGround(elevator, arm, gripper));
    leftPaddle.onFalse(new RaiseBeforeStow(elevator, arm, gripper));
    // Scoring
    buttonB.onTrue(
        new InstantCommand(
            () -> {
              aState.setScoringLevel(ReefLevel.ONE);
            }));
    buttonA.onTrue(
        new InstantCommand(
            () -> {
              aState.setScoringLevel(ReefLevel.TWO);
            }));
    buttonX.onTrue(
        new InstantCommand(
            () -> {
              aState.setScoringLevel(ReefLevel.THREE);
            }));
    buttonY.onTrue(
        new InstantCommand(
            () -> {
              aState.setScoringLevel(ReefLevel.FOUR);
            }));

    bumperLeft.whileTrue(
        new ScoreCoralCommand(ReefStackChoice.LEFT, swerve, arm, elevator, gripper));
    bumperLeft.onFalse(new InstantCommand(() -> swerve.trackAllTags()));
    bumperRight.whileTrue(
        new ScoreCoralCommand(ReefStackChoice.RIGHT, swerve, arm, elevator, gripper));
    bumperRight.onFalse(new InstantCommand(() -> swerve.trackAllTags()));

    leftTrigger.whileTrue(new IntakeOrScoreAlgaeCommand(swerve, arm, elevator, gripper, true));
    rightTrigger.whileTrue(new IntakeOrScoreAlgaeCommand(swerve, arm, elevator, gripper, false));

    dPadUp.whileTrue(new EmergencyUntangle(elevator, arm, gripper));
    dPadDown.whileTrue(
        new RepeatCommand(
            new InstantCommand(
                () -> {
                  gripper.scoreL4();
                  gripper.hasAlgae = false;
                  climber.isCliming = false;
                  if (Robot.isSimulation()) {
                    SimulatedArena.getInstance().resetFieldForAuto();
                  }
                })));

    dPadLeft.whileTrue(
        new InstantCommand(
            () -> {
              climber.deployMotor();
              climber.isCliming = true;
            }));
    dPadLeft.onFalse(new InstantCommand(() -> climber.stop()));
    dPadRight.whileTrue(
        new InstantCommand(
            () -> {
              climber.isCliming = true;
              climber.retractMotor();
            }));
    dPadRight.onFalse(new InstantCommand(() -> climber.stop()));

    buttonBack.whileTrue(new RepeatCommand(new InstantCommand(() -> funnel.reverseTop())));
  }

  public void disabledPeriodic() {
    final var selectedAuto = autoChooser.get();
    if (selectedAuto == null) {
      // Clear poses
      fieldViz.getObject("traj").setPoses();
      LoggerHelper.recordPose2dList("AutoTraj", new ArrayList<Pose2d>());
    } else if (lastSelectedAuto != selectedAuto) {
      selectedAuto.loadAndUpdateViz(fieldViz);
      swerve.resetPose(selectedAuto.getInitialPose());
      if (Robot.isSimulation()) {
        swerve.resetSimPose(selectedAuto.getInitialPose());
      }
    }
    lastSelectedAuto = selectedAuto;

    driverXbox.setRumble(RumbleType.kBothRumble, 0);
  }

  public void robotPeriodic() {
    // Update canbus inputs
    rioBus.updateInputs();
    canivoreBus.updateInputs();

    // 3d viz
    final ScoringKinematicsOutput kinematicsOutput =
        ScoringKinematics.computeForwardKinematics(
            elevator.getHeight(), arm.getAngle(), gripper.getWristAngle());
    Logger.recordOutput(
        "mechanismPoses",
        new Pose3d[] {
          kinematicsOutput.getElevatorPose(),
          kinematicsOutput.getArmPose(),
          kinematicsOutput.getGripperPose()
        });

    Logger.recordOutput("ReefLevel/L1", aState.getScoringLevel() == ReefLevel.ONE);
    Logger.recordOutput("ReefLevel/L2", aState.getScoringLevel() == ReefLevel.TWO);
    Logger.recordOutput("ReefLevel/L3", aState.getScoringLevel() == ReefLevel.THREE);
    Logger.recordOutput("ReefLevel/L4", aState.getScoringLevel() == ReefLevel.FOUR);
  }

  public void teleopPeriodic() {
    driverXbox.setRumble(
        RumbleType.kBothRumble,
        AlignmentUtilities.rumbleCondition(
                swerve.getSingleTagPose(),
                DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Blue)
            ? 1
            : 0);
  }

  public Command getDisabledCommand() {
    return null;
  }

  public Command getAutonomousCommand() {
    final var selectedAuto = autoChooser.get();
    if (selectedAuto == null) {
      return null;
    }
    return selectedAuto.getCommand();
  }

  public void simulationPeriodic() {
    // Run MapleSim tick
    final var simulatedArena = SimulatedArena.getInstance();
    simulatedArena.simulationPeriodic();
    Logger.recordOutput("FieldSimulation/Algae", simulatedArena.getGamePiecesArrayByType("Algae"));
    Logger.recordOutput("FieldSimulation/Coral", simulatedArena.getGamePiecesArrayByType("Coral"));

    // Simulate loading
    final var alliance = DriverStation.getAlliance();
    final Fiducial[] loadingStationTagIDs =
        (alliance.isPresent() && alliance.get() == Alliance.Blue)
            ? Fiducials.blueCoralStationTags
            : Fiducials.redCoralStationTags;
    for (Fiducial tag : loadingStationTagIDs) {
      final Pose2d robotInTagFrame = swerve.getPose().relativeTo(tag.getPose().toPose2d());
      if (robotInTagFrame.getX() < 1.0
          && Math.abs(robotInTagFrame.getRotation().getRadians()) < Math.toRadians(60.0)) {
        gripper.setSimHasCoral(true);
        break;
      }
    }

    // Viz game piece in gripper
    final ScoringKinematicsOutput kinematicsOutput =
        ScoringKinematics.computeForwardKinematics(
            elevator.getHeight(), arm.getAngle(), gripper.getWristAngle());
    Logger.recordOutput(
        "FieldSimulation/Gripper Algae",
        gripper.simHasAlgae()
            ? new Pose3d(swerve.getPose()).transformBy(kinematicsOutput.algaeT)
            : Pose3d.kZero);
    Logger.recordOutput(
        "FieldSimulation/Gripper Coral",
        gripper.simHasCoral()
            ? new Pose3d(swerve.getPose()).transformBy(kinematicsOutput.coralT)
            : Pose3d.kZero);
  }
}
