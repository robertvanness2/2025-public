// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.advantagekit.LoggerHelper;
import frc.quixlib.math.MathUtils;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.robot.Constants;
import frc.robot.Robot;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  private final QuixTalonFX m_leftMotor =
      new QuixTalonFX(
          Constants.Elevator.leftMotorID,
          Constants.Elevator.motorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setPIDConfig(Constants.Elevator.positionSlot, Constants.Elevator.elevatorPIDConfig)
              .setInverted(Constants.Elevator.leftMotorInvert)
              .setBrakeMode()
              // Not sure how to simulate two motors in sim properly.
              // Double the current limits instead.
              .setSupplyCurrentLimit(Robot.isSimulation() ? 80.0 : 40.0)
              .setStatorCurrentLimit(Robot.isSimulation() ? 160.0 : 80.0)
              .setRotorBootOffset(Constants.Elevator.rotorBootOffset)
              .setBootPositionOffset(Constants.Elevator.startingHeight)
              .setReverseSoftLimit(Constants.Elevator.minHeight)
              .setForwardSoftLimit(Constants.Elevator.maxHeight));

  private final QuixTalonFX m_rightMotor =
      new QuixTalonFX(
          Constants.Elevator.rightMotorID,
          m_leftMotor,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Elevator.rightMotorInvert)
              .setBrakeMode()
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(80.0));

  private double m_targetHeight = Constants.Elevator.startingHeight;
  private boolean m_isGentle = false;

  public ElevatorSubsystem() {}

  public double getHeight() {
    return m_leftMotor.getSensorPosition();
  }

  public void setHeight(double height, boolean isGentle) {
    m_targetHeight =
        MathUtils.clamp(height, Constants.Elevator.minHeight, Constants.Elevator.maxHeight);
    m_isGentle = isGentle;
  }

  public boolean isElevatorAtHeight(double height, double tolerance) {
    return Math.abs(height - m_leftMotor.getSensorPosition()) <= tolerance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    LoggerHelper.recordCurrentCommand(this);
    m_leftMotor.updateInputs();
    m_rightMotor.updateInputs();

    m_leftMotor.setDynamicMotionMagicPositionSetpoint(
        Constants.Elevator.positionSlot,
        m_targetHeight,
        m_isGentle ? Constants.Elevator.maxGentleVelocity : Constants.Elevator.maxVelocity,
        m_isGentle ? Constants.Elevator.maxGentleAcceleration : Constants.Elevator.maxAcceleration,
        Constants.Elevator.maxJerk);

    Logger.recordOutput(
        "Elevator/Current Height (inches)", Units.metersToInches(m_leftMotor.getSensorPosition()));
    Logger.recordOutput(
        "Elevator/Target Height (inches)",
        Units.metersToInches(m_leftMotor.getClosedLoopReference()));
    Logger.recordOutput(
        "Elevator/Current Velocity (inches per s)",
        Units.metersToInches(m_leftMotor.getSensorVelocity()));
    Logger.recordOutput(
        "Elevator/Target Velocity (inches per s)",
        Units.metersToInches(m_leftMotor.getClosedLoopReferenceSlope()));
    Logger.recordOutput("Elevator/Is Gentle", m_isGentle);
  }

  // --- BEGIN STUFF FOR SIMULATION ---
  private static final ElevatorSim m_elevatorSim =
      new ElevatorSim(
          DCMotor.getKrakenX60Foc(2),
          Constants.Elevator.motorRatio.reduction(),
          Constants.Elevator.simCarriageMass,
          Constants.Elevator.sprocketPitchDiameter * 0.5,
          Constants.Elevator.minHeight,
          Constants.Elevator.maxHeight,
          true,
          Constants.Elevator.startingHeight);

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

    m_elevatorSim.setInput(m_leftMotor.getPercentOutput() * RobotController.getBatteryVoltage());
    m_elevatorSim.update(LoggedRobot.defaultPeriodSecs);
    m_leftMotor.setSimSensorPositionAndVelocity(
        m_elevatorSim.getPositionMeters(),
        m_elevatorSim.getVelocityMetersPerSecond(),
        LoggedRobot.defaultPeriodSecs,
        Constants.Elevator.motorRatio);
  }
  // --- END STUFF FOR SIMULATION ---
}
