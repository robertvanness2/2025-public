// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.advantagekit.LoggerHelper;
import frc.quixlib.math.MathUtils;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.robot.Constants;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
  private final QuixTalonFX m_rightMotor =
      new QuixTalonFX(
          Constants.Arm.rightMotorID,
          Constants.Arm.armRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Arm.rightMotorInvert)
              .setBrakeMode()
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(100.0)
              .setPIDConfig(Constants.Arm.positionSlot, Constants.Arm.PIDConfig)
              .setRotorBootOffset(Constants.Arm.rotorBootOffset)
              .setBootPositionOffset(Constants.Arm.startingAngle)
              .setReverseSoftLimit(Constants.Arm.minAngle)
              .setForwardSoftLimit(Constants.Arm.maxAngle));

  private double m_targetAngle = Constants.Arm.stowAngle;
  private boolean m_isGentle = false;

  public ArmSubsystem() {}

  public void setAngle(double angle, boolean isGentle) {
    m_targetAngle = MathUtils.clamp(angle, Constants.Arm.minAngle, Constants.Arm.maxAngle);
    m_isGentle = isGentle;
  }

  public double getAngle() {
    return m_rightMotor.getSensorPosition();
  }

  public boolean isArmAtAngle(double angle, double tolerance) {
    return Math.abs(angle - m_rightMotor.getSensorPosition()) <= tolerance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    LoggerHelper.recordCurrentCommand(this);
    m_rightMotor.updateInputs();

    m_rightMotor.setDynamicMotionMagicPositionSetpoint(
        Constants.Arm.positionSlot,
        m_targetAngle,
        Constants.Arm.maxVelocity,
        m_isGentle ? Constants.Arm.maxGentleAcceleration : Constants.Arm.maxAcceleration,
        Constants.Arm.maxJerk);

    Logger.recordOutput(
        "Arm/Current Angle (deg)", Units.radiansToDegrees(m_rightMotor.getSensorPosition()));
    Logger.recordOutput(
        "Arm/Target Angle (deg)", Units.radiansToDegrees(m_rightMotor.getClosedLoopReference()));
    Logger.recordOutput(
        "Arm/Current Velocity (deg per sec)",
        Units.radiansToDegrees(m_rightMotor.getSensorVelocity()));
    Logger.recordOutput(
        "Arm/Target Velocity (deg per sec)",
        Units.radiansToDegrees(m_rightMotor.getClosedLoopReferenceSlope()));
    Logger.recordOutput("Arm/Is Gentle", m_isGentle);
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private static final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          Constants.Arm.armRatio.reduction(),
          Constants.Arm.simArmMOI,
          Constants.Arm.simArmCGLength,
          Constants.Arm.minAngle,
          Constants.Arm.maxAngle,
          true, // Simulate gravity
          Constants.Arm.startingAngle);

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    m_armSim.setInput(m_rightMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_armSim.update(LoggedRobot.defaultPeriodSecs);
    m_rightMotor.setSimSensorPositionAndVelocity(
        m_armSim.getAngleRads() - Constants.Arm.startingAngle,
        m_armSim.getVelocityRadPerSec(),
        LoggedRobot.defaultPeriodSecs,
        Constants.Arm.armRatio);
  }
  // --- END STUFF FOR SIMULATION ---
}
