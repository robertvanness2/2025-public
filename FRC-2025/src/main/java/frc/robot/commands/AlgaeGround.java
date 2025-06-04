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

public class AlgaeGround extends Command {
  private final ElevatorSubsystem m_elevator;
  private final GripperSubsystem m_gripper;
  private final ArmSubsystem m_arm;

  public AlgaeGround(ElevatorSubsystem elevator, ArmSubsystem arm, GripperSubsystem gripper) {
    m_elevator = elevator;
    m_arm = arm;
    m_gripper = gripper;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, arm, gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setHeight(Constants.Elevator.minHeight, false);
    m_arm.setAngle(Units.degreesToRadians(-49.0), false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_gripper.holdAlgae();
    if (m_arm.isArmAtAngle(Units.degreesToRadians(-49.0), Units.degreesToRadians(25.0))) {
      m_gripper.setWristAngle(Units.degreesToRadians(7.5));
    } else {
      m_gripper.setWristAngle(Constants.Gripper.maxAngle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_gripper.hasAlgae = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
