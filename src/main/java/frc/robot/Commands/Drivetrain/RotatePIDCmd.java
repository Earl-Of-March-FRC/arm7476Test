// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DrivetrainConstants.PIDConstants;
import frc.robot.Subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotatePIDCmd extends PIDCommand {
  /** Creates a new RotatePIDCmd. */
  public RotatePIDCmd(DrivetrainSubsystem driveSub, DoubleSupplier kP, DoubleSupplier kI, DoubleSupplier kD, DoubleSupplier setpoint) {
    super(
        // The controller that the command will use
        new PIDController(kP.getAsDouble(), kI.getAsDouble(), kD.getAsDouble()),
        // This should return the measurement
        () -> driveSub.getGyroPID(),
        // This should return the setpoint (can also be a constant)
        () -> setpoint.getAsDouble(),
        // This uses the output
        output -> {
          driveSub.setPermanent(0, 0, MathUtil.clamp(-PIDConstants.rotClamp, PIDConstants.rotClamp, output));
          SmartDashboard.putNumber("Rot Output", MathUtil.clamp(-PIDConstants.rotClamp, PIDConstants.rotClamp, output));
          SmartDashboard.putNumber("Rot Measurement", driveSub.getGyroPID());
          System.out.println("P: " + kP.getAsDouble());
          System.out.println("I: " + kI.getAsDouble());
          System.out.println("D: " + kD.getAsDouble());
          System.out.println("Setpoint: " + setpoint.getAsDouble());
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(driveSub);
    getController().setTolerance(PIDConstants.rotTolerance);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
