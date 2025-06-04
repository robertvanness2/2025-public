// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class RaiseBeforeStow extends Command {
  private final ElevatorSubsystem m_elevator;
  private final GripperSubsystem m_gripper;
  private final ArmSubsystem m_arm;

  public RaiseBeforeStow(ElevatorSubsystem elevator, ArmSubsystem arm, GripperSubsystem gripper) {
    m_elevator = elevator;
    m_arm = arm;
    m_gripper = gripper;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, arm, gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setHeight(Units.inchesToMeters(15.0), true);
    if (m_elevator.isElevatorAtHeight(Units.inchesToMeters(15.0), Units.inchesToMeters(2.0))) {
      m_arm.setAngle(Constants.Arm.stowAlgaeAngle, true);
      m_gripper.setWristAngle(Constants.Gripper.stowAlgaeAngle);
    }
    m_gripper.holdAlgae();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.isElevatorAtHeight(Units.inchesToMeters(15.0), Units.inchesToMeters(1.0));
  }
}
