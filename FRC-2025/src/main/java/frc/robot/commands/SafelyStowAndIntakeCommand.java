// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AlignmentUtilities;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import java.util.function.Supplier;

public class SafelyStowAndIntakeCommand extends Command {
  private final ElevatorSubsystem m_elevator;
  private final ArmSubsystem m_arm;
  private final GripperSubsystem m_gripper;
  private final Supplier<Pose2d> m_robotPose;

  public SafelyStowAndIntakeCommand(
      ElevatorSubsystem elevator,
      ArmSubsystem arm,
      GripperSubsystem gripper,
      Supplier<Pose2d> robotPose) {
    m_elevator = elevator;
    m_arm = arm;
    m_gripper = gripper;
    m_robotPose = robotPose;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, arm, gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (AlignmentUtilities.isClearOfReef(
        m_robotPose.get(),
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Blue)) {
      if (m_gripper.hasAlgae) {
        m_elevator.setHeight(Constants.Elevator.stowAlgaeHeight, true);
        m_arm.setAngle(Constants.Arm.stowAlgaeAngle, true);
        m_gripper.setWristAngle(Constants.Gripper.stowAlgaeAngle);
      } else {
        m_elevator.setHeight(Constants.Elevator.stowHeight, false);
        m_arm.setAngle(Constants.Arm.stowAngle, true);
        m_gripper.setWristAngle(Constants.Gripper.stowAngle);
      }
    } else {
      if (m_arm.getAngle() > Constants.Arm.levelFourReefClearanceAngle) {
        m_elevator.setHeight(Constants.Elevator.stowHeight, false);
      }
    }

    if (m_gripper.hasAlgae) {
      m_gripper.holdAlgae();
    } else if (!m_gripper.hasPiece()) {
      m_gripper.intake();
    } else {
      m_gripper.stopRollers();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
