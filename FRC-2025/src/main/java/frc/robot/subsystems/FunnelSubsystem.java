package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.quixlib.advantagekit.LoggerHelper;
import frc.quixlib.motorcontrol.QuixTalonFX;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class FunnelSubsystem extends SubsystemBase {
  private final QuixTalonFX m_funnelMotor =
      new QuixTalonFX(
          Constants.Funnel.funnelMotorID,
          Constants.Funnel.funnelMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Funnel.funnelMotorInvert)
              .setBrakeMode()
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(40.0));

  private final QuixTalonFX m_topRollerMotor =
      new QuixTalonFX(
          Constants.Funnel.topRollerMotorID,
          Constants.Funnel.topRollerMotorRatio,
          QuixTalonFX.makeDefaultConfig()
              .setInverted(Constants.Funnel.topRollerMotorInvert)
              .setBrakeMode()
              .setSupplyCurrentLimit(40.0)
              .setStatorCurrentLimit(40.0));

  public FunnelSubsystem() {}

  public void intake() {
    m_funnelMotor.setPercentOutput(0.5);
    m_topRollerMotor.setPercentOutput(0.5);
  }

  public void reverseTop() {
    m_topRollerMotor.setPercentOutput(-1.0);
  }

  public void stop() {
    m_funnelMotor.setPercentOutput(0.0);
    m_topRollerMotor.setPercentOutput(0.0);
  }

  @Override
  public void periodic() {
    LoggerHelper.recordCurrentCommand(this);
    m_funnelMotor.updateInputs();
    m_topRollerMotor.updateInputs();

    Logger.recordOutput(
        "Funnel/Roller Current Velocity (rads per sec)", m_funnelMotor.getSensorVelocity());
    Logger.recordOutput(
        "Funnel/Roller Target Velocity (rads per sec)", m_funnelMotor.getClosedLoopReference());

    Logger.recordOutput(
        "Funnel/Top Roller Current Velocity (rads per sec)", m_topRollerMotor.getSensorVelocity());
    Logger.recordOutput(
        "Funnel/Top Roller Target Velocity (rads per sec)",
        m_topRollerMotor.getClosedLoopReference());
  }

  @Override
  public void simulationPeriodic() {}
}
