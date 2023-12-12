// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class MecanumDriveCmd extends CommandBase {
  private DrivetrainSubsystem driveSub;
  private DoubleSupplier sxFunc, syFunc, rxFunc, scaleFactor;
  private BooleanSupplier limit;

  private SlewRateLimiter sxFilter, syFilter, rxFilter;

  // private final SlewRateLimiter sxFilter = new SlewRateLimiter(DrivetrainConstants.slewRate);
  // private final SlewRateLimiter syFilter = new SlewRateLimiter(DrivetrainConstants.slewRate);
  // private final SlewRateLimiter rxFilter = new SlewRateLimiter(DrivetrainConstants.slewRate);

  /** Creates a new MecanumDriveCmd. */
  public MecanumDriveCmd(
    DrivetrainSubsystem driveSub,
    DoubleSupplier xSpeed,
    DoubleSupplier ySpeed,
    DoubleSupplier xRot,
    DoubleSupplier scaling,
    BooleanSupplier limit,
    SlewRateLimiter sxFilter, SlewRateLimiter syFilter, SlewRateLimiter rxFilter
  ) {
    this.driveSub = driveSub;
    this.sxFunc = xSpeed;
    this.syFunc = ySpeed;
    this.rxFunc = xRot;
    this.scaleFactor = scaling;
    this.limit = limit;
    this.sxFilter = sxFilter;
    this.syFilter = syFilter;
    this.rxFilter = rxFilter;

    addRequirements(driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Double forward, side, rotate, scale;
    scale = MathUtil.applyDeadband(scaleFactor.getAsDouble(), DriverConstants.limitDeadband);

    forward = (DrivetrainConstants.maxDriveSpeed * (-scale + 1) / 2) * sxFunc.getAsDouble();
    side = (DrivetrainConstants.maxDriveSpeed * (-scale + 1) / 2) * syFunc.getAsDouble();
    rotate = (DrivetrainConstants.maxTurnSpeed * (-scale + 1) / 2) * rxFunc.getAsDouble();

    if (limit.getAsBoolean()) { // If limit is true, it will prevent the robot from moving forward/backwards and/or rotate
      forward = 0.0;
      rotate = 0.0;
    }

    driveSub.driveFieldOriented(
      MathUtil.applyDeadband(sxFilter.calculate(forward), DriverConstants.deadband),
      MathUtil.applyDeadband(syFilter.calculate(side), DriverConstants.deadband),
      MathUtil.applyDeadband(rxFilter.calculate(rotate), DriverConstants.deadband)
    );
    SmartDashboard.putNumber("Forward", sxFunc.getAsDouble());
    SmartDashboard.putNumber("Side", syFunc.getAsDouble());
    SmartDashboard.putNumber("Rotation", rxFunc.getAsDouble());
    SmartDashboard.putNumber("Scale Factor", scaleFactor.getAsDouble());
    SmartDashboard.putNumber("Motor Scale", scale);
    SmartDashboard.putBoolean("Limit", limit.getAsBoolean());
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