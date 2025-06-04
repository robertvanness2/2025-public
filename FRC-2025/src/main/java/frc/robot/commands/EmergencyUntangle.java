// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class EmergencyUntangle extends Command {
  private final ElevatorSubsystem m_elevator;
  private final ArmSubsystem m_arm;
  private final GripperSubsystem m_gripper;

  public EmergencyUntangle(ElevatorSubsystem elevator, ArmSubsystem arm, GripperSubsystem gripper) {
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
    m_elevator.setHeight(Constants.Elevator.maxHeight, true);
    m_arm.setAngle(Constants.Arm.untangleAngle, true);
    m_gripper.setWristAngle(Constants.Gripper.untangleAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.setHeight(Constants.Elevator.stowHeight, true);
    m_arm.setAngle(Constants.Arm.stowAngle, true);
    m_gripper.setWristAngle(Constants.Gripper.stowAngle);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
