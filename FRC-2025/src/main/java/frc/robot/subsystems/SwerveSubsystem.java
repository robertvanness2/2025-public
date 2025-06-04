// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.quixlib.devices.QuixPigeon2;
import frc.quixlib.swerve.QuixSwerve;
import frc.quixlib.swerve.QuixSwerveModule;
import frc.quixlib.swerve.QuixSwerveModuleFactory;
import frc.quixlib.vision.QuixVisionCamera;
import frc.quixlib.vision.QuixVisionSim;
import frc.robot.Constants;
import frc.robot.Fiducials;
import java.util.ArrayList;

public class SwerveSubsystem extends QuixSwerve {
  public SwerveSubsystem(
      QuixPigeon2 imu,
      ArrayList<QuixVisionCamera> cameras,
      QuixVisionSim visionSim,
      Field2d fieldViz) {
    super(
        imu,
        cameras,
        Constants.Swerve.maxDriveSpeed,
        Constants.Swerve.maxModuleAcceleration,
        Constants.Swerve.maxModuleSteeringRate,
        Constants.Swerve.driveController,
        Fiducials.aprilTagFiducials,
        visionSim,
        fieldViz);
  }

  protected QuixSwerveModule[] createModules() {
    QuixSwerveModuleFactory swerveModuleFactory =
        new QuixSwerveModuleFactory(
            Constants.Swerve.wheelCircumference,
            Constants.Swerve.driveRatio,
            Constants.Swerve.steeringRatio,
            Constants.Swerve.steerDriveCouplingRatio,
            Constants.Swerve.driveOpenLoopPIDConfig,
            Constants.Swerve.driveClosedLoopPIDConfig,
            Constants.Swerve.steeringPIDConfig);
    QuixSwerveModule[] modules = {
      swerveModuleFactory.createModule(
          Constants.Swerve.FrontLeft.modulePosition,
          Constants.Swerve.FrontLeft.driveMotorID,
          Constants.Swerve.FrontLeft.steeringMotorID,
          Constants.Swerve.FrontLeft.canCoderID,
          Constants.Swerve.FrontLeft.absEncoderOffsetRad),
      swerveModuleFactory.createModule(
          Constants.Swerve.RearLeft.modulePosition,
          Constants.Swerve.RearLeft.driveMotorID,
          Constants.Swerve.RearLeft.steeringMotorID,
          Constants.Swerve.RearLeft.canCoderID,
          Constants.Swerve.RearLeft.absEncoderOffsetRad),
      swerveModuleFactory.createModule(
          Constants.Swerve.RearRight.modulePosition,
          Constants.Swerve.RearRight.driveMotorID,
          Constants.Swerve.RearRight.steeringMotorID,
          Constants.Swerve.RearRight.canCoderID,
          Constants.Swerve.RearRight.absEncoderOffsetRad),
      swerveModuleFactory.createModule(
          Constants.Swerve.FrontRight.modulePosition,
          Constants.Swerve.FrontRight.driveMotorID,
          Constants.Swerve.FrontRight.steeringMotorID,
          Constants.Swerve.FrontRight.canCoderID,
          Constants.Swerve.FrontRight.absEncoderOffsetRad),
    };
    return modules;
  }
}
