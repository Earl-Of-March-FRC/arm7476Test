// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class MecanumDriveRawCmd extends CommandBase {
  private DrivetrainSubsystem driveSub;
  private DoubleSupplier xSpeed, ySpeed, xRot;

  /** Creates a new MecanumDrivePermanentCmd. */
  public MecanumDriveRawCmd(
    DrivetrainSubsystem driveSub,
    DoubleSupplier xSpeed,
    DoubleSupplier ySpeed,
    DoubleSupplier xRot
  ) {
    this.driveSub = driveSub;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.xRot = xRot;

    addRequirements(driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSub.driveFieldOriented(xSpeed.getAsDouble(), ySpeed.getAsDouble(), xRot.getAsDouble());
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
