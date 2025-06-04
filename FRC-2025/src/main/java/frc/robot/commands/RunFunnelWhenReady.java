// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FunnelSubsystem;
import java.util.function.Supplier;

public class RunFunnelWhenReady extends Command {
  private final FunnelSubsystem m_funnel;
  private final Supplier<Boolean> m_runFunnel;

  public RunFunnelWhenReady(FunnelSubsystem funnel, Supplier<Boolean> runFunnel) {
    m_funnel = funnel;
    m_runFunnel = runFunnel;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(funnel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_runFunnel.get()) {
      m_funnel.intake();
    } else {
      m_funnel.stop();
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
