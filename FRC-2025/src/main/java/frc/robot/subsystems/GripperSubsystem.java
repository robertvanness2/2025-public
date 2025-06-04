// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.advantagekit.LoggerHelper;
import frc.quixlib.math.MathUtils;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.quixlib.wpilib.LoggedDigitalInput;
import frc.robot.Constants;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

public class GripperSubsystem extends SubsystemBase {
  private final QuixTalonFX m_leftRollerMotor =
      new QuixTalonFX(
          Constants.Gripper.leftRollerMotorID,
          Constants.Gripper.rollerMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Gripper.leftRollerMotorInvert)
              .setBrakeMode()
              .setSupplyCurrentLimit(20.0)
              .setStatorCurrentLimit(50.0)
              .setPIDConfig(
                  Constants.Gripper.leftRollerVelocitySlot,
                  Constants.Gripper.rollerVelocityPIDConfig));

  private final QuixTalonFX m_rightRollerMotor =
      new QuixTalonFX(
          Constants.Gripper.rightRollerMotorID,
          Constants.Gripper.rollerMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Gripper.rightRollerMotorInvert)
              .setBrakeMode()
              .setSupplyCurrentLimit(20.0)
              .setStatorCurrentLimit(50.0)
              .setPIDConfig(
                  Constants.Gripper.rightRollerVelocitySlot,
                  Constants.Gripper.rollerVelocityPIDConfig));

  private final QuixTalonFX m_wristMotor =
      new QuixTalonFX(
          Constants.Gripper.wristMotorID,
          Constants.Gripper.wristMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Gripper.wristMotorInvert)
              .setBrakeMode()
              .setSupplyCurrentLimit(20.0)
              .setStatorCurrentLimit(60.0)
              .setMotionMagicConfig(
                  Constants.Gripper.wristMaxVelocity,
                  Constants.Gripper.wristMaxAcceleration,
                  Constants.Gripper.wristMaxJerk)
              .setPIDConfig(
                  Constants.Gripper.wristPositionSlot, Constants.Gripper.wristPositionPIDConfig)
              .setRotorBootOffset(Constants.Gripper.rotorBootOffset)
              .setBootPositionOffset(Constants.Gripper.startingAngle)
              .setReverseSoftLimit(Constants.Gripper.minAngle)
              .setForwardSoftLimit(Constants.Gripper.maxAngle));

  private final LoggedDigitalInput m_beamBreak =
      new LoggedDigitalInput(Constants.Gripper.beamBreakPort);

  private double m_targetAngle = Constants.Gripper.startingAngle;

  public boolean hasAlgae = false;

  private Timer m_noAlgaeTimer = new Timer();

  private boolean m_simHasAlgae = false;
  private boolean m_simHasCoral = false;

  public GripperSubsystem() {
    m_noAlgaeTimer.start();
  }

  public void scoreL4() {
    m_leftRollerMotor.setVelocitySetpoint(
        Constants.Gripper.leftRollerVelocitySlot, Constants.Gripper.scoreCoralVelocityL4);
    m_rightRollerMotor.setVelocitySetpoint(
        Constants.Gripper.rightRollerVelocitySlot, Constants.Gripper.scoreCoralVelocityL4);
  }

  public void scoreL3L2() {
    m_leftRollerMotor.setVelocitySetpoint(
        Constants.Gripper.leftRollerVelocitySlot, Constants.Gripper.scoreCoralVelocityL3L2);
    m_rightRollerMotor.setVelocitySetpoint(
        Constants.Gripper.rightRollerVelocitySlot, Constants.Gripper.scoreCoralVelocityL3L2);
  }

  public void scoreL1Straight() {
    m_leftRollerMotor.setVelocitySetpoint(
        Constants.Gripper.leftRollerVelocitySlot, Constants.Gripper.scoreCoralVelocityL1Straight);
    m_rightRollerMotor.setVelocitySetpoint(
        Constants.Gripper.rightRollerVelocitySlot, Constants.Gripper.scoreCoralVelocityL1Straight);
  }

  public void scoreLeft() {
    // m_leftRollerMotor.setCurrentOutput(-3.0, 0.3);
    m_leftRollerMotor.setVelocitySetpoint(Constants.Gripper.rightRollerVelocitySlot, 20);
    m_rightRollerMotor.setVelocitySetpoint(
        Constants.Gripper.rightRollerVelocitySlot, Constants.Gripper.scoreCoralVelocityL1);
  }

  public void scoreRight() {
    m_leftRollerMotor.setVelocitySetpoint(
        Constants.Gripper.leftRollerVelocitySlot, Constants.Gripper.scoreCoralVelocityL1);
    m_rightRollerMotor.setVelocitySetpoint(Constants.Gripper.leftRollerVelocitySlot, 20);
    // m_rightRollerMotor.setCurrentOutput(-3.0, 0.3);
  }

  public void intake() {
    m_leftRollerMotor.setVelocitySetpoint(
        Constants.Gripper.leftRollerVelocitySlot, Constants.Gripper.intakeVelocity);
    m_rightRollerMotor.setVelocitySetpoint(
        Constants.Gripper.rightRollerVelocitySlot, Constants.Gripper.intakeVelocity);
  }

  public void holdAlgae() {
    m_leftRollerMotor.setCurrentOutput(-50.0, 0.5);
    m_rightRollerMotor.setCurrentOutput(-50.0, 0.5);
  }

  public void scoreAlgae() {
    m_leftRollerMotor.setPercentOutput(1.0);
    m_rightRollerMotor.setPercentOutput(1.0);
  }

  public void stopRollers() {
    m_leftRollerMotor.setPercentOutput(0.0);
    m_rightRollerMotor.setPercentOutput(0.0);
  }

  public void setWristAngle(double angle) {
    m_targetAngle = MathUtils.clamp(angle, Constants.Gripper.minAngle, Constants.Gripper.maxAngle);
  }

  public double getWristAngle() {
    return m_wristMotor.getSensorPosition();
  }

  public boolean isWristAtAngle(double angle, double tolerance) {
    return Math.abs(angle - m_wristMotor.getSensorPosition()) <= tolerance;
  }

  public boolean simHasAlgae() {
    return m_simHasAlgae;
  }

  public boolean simHasCoral() {
    return m_simHasCoral;
  }

  public boolean hasPiece() {
    return m_beamBreak.get();
  }

  public void setSimHasAlgae(boolean hasAlgae) {
    m_simHasAlgae = hasAlgae;
  }

  public void setSimHasCoral(boolean hasCoral) {
    m_simHasCoral = hasCoral;
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    // Free spin at -100 rad/s
    if (m_leftRollerMotor.getSensorVelocity() > -80.0
        || m_rightRollerMotor.getSensorVelocity() > -80.0) {
      m_noAlgaeTimer.reset();
    }
    if (m_noAlgaeTimer.get() > 0.5) {
      hasAlgae = false;
    }

    LoggerHelper.recordCurrentCommand(this);
    m_leftRollerMotor.updateInputs();
    m_rightRollerMotor.updateInputs();
    m_wristMotor.updateInputs();
    m_beamBreak.updateInputs();

    m_wristMotor.setMotionMagicPositionSetpoint(Constants.Gripper.wristPositionSlot, m_targetAngle);
    Logger.recordOutput(
        "Wrist/Current Angle (deg)", Units.radiansToDegrees(m_wristMotor.getSensorPosition()));
    Logger.recordOutput(
        "Wrist/Target Angle (deg)", Units.radiansToDegrees(m_wristMotor.getClosedLoopReference()));
    Logger.recordOutput(
        "Wrist/Current Velocity (deg per sec)",
        Units.radiansToDegrees(m_wristMotor.getSensorVelocity()));
    Logger.recordOutput(
        "Wrist/Target Velocity (deg per sec)",
        Units.radiansToDegrees(m_wristMotor.getClosedLoopReferenceSlope()));

    Logger.recordOutput(
        "Gripper/Left Roller Current Velocity (rads per sec)",
        m_leftRollerMotor.getSensorVelocity());
    Logger.recordOutput(
        "Gripper/Left Roller Target Velocity (rads per sec)",
        m_leftRollerMotor.getClosedLoopReference());
    Logger.recordOutput(
        "Gripper/Right Roller Current Velocity (rads per sec)",
        m_rightRollerMotor.getSensorVelocity());
    Logger.recordOutput(
        "Gripper/Right Roller Target Velocity (rads per sec)",
        m_rightRollerMotor.getClosedLoopReference());

    Logger.recordOutput("Gripper/Has Algae", hasAlgae);
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private static final SingleJointedArmSim m_wristSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          Constants.Gripper.wristMotorRatio.reduction(),
          Constants.Gripper.simWristMOI,
          Constants.Gripper.simWristCGLength,
          Constants.Gripper.minAngle,
          Constants.Gripper.maxAngle,
          // Don't simulate gravity because the wrist is mounted on the arm and it becomes
          // incorrect. This shouldn't be a big deal because the wrist is low-torque.
          // TODO: Properly simulate gravity
          false,
          Constants.Gripper.startingAngle);

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_beamBreak.setSimValue(m_simHasCoral);

    m_wristSim.setInput(m_wristMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_wristSim.update(LoggedRobot.defaultPeriodSecs);
    m_wristMotor.setSimSensorPositionAndVelocity(
        m_wristSim.getAngleRads() - Constants.Gripper.startingAngle,
        m_wristSim.getVelocityRadPerSec(),
        LoggedRobot.defaultPeriodSecs,
        Constants.Gripper.wristMotorRatio);
  }
  // --- END STUFF FOR SIMULATION ---
}
