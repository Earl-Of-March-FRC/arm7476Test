// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drivetrain.Autos;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Drivetrain.MecanumDriveRawCmd;
import frc.robot.Commands.Drivetrain.RotatePIDCmd;
import frc.robot.Subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveAndTurn extends SequentialCommandGroup {
  /** Creates a new TurnAndMove. */
  public MoveAndTurn(
    DrivetrainSubsystem driveSub,
    double kP, double kI, double kD,
    double initialAngle,
    SlewRateLimiter sxFilter, SlewRateLimiter syFilter, SlewRateLimiter rxFilter
    ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MecanumDriveRawCmd(driveSub, () -> 0.1, () -> 0, () -> 0.1).withTimeout(2),
      new MecanumDriveRawCmd(driveSub, () -> -0.1, () -> 0, () -> -0.1).withTimeout(2),
      new RotatePIDCmd(driveSub, () -> kP, () -> kI, () -> kD, () -> initialAngle)
    );
  }
}
