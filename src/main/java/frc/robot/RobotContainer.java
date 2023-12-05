// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.Drivetrain.MecanumDriveCmd;
import frc.robot.Commands.Drivetrain.RotatePIDCmd;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.DrivetrainConstants.PIDConstants;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class RobotContainer {
  private final DrivetrainSubsystem driveSub = new DrivetrainSubsystem();

  private final XboxController controller = new XboxController(DriverConstants.port);
  private final CommandXboxController commandController = new CommandXboxController(DriverConstants.port);
  
  public RobotContainer() {
    driveSub.setDefaultCommand(
      new MecanumDriveCmd(
        driveSub,
        () -> controller.getRawAxis(DriverConstants.forwardAxis),
        () -> controller.getRawAxis(DriverConstants.sideAxis),
        () -> controller.getRawAxis(DriverConstants.rotAxis),
        () -> controller.getRawAxis(DriverConstants.scalingAxis),
        () -> controller.getRightBumper()
      )
    );

    SmartDashboard.putNumber("Rot P", PIDConstants.rotP);
    SmartDashboard.putNumber("Rot I", PIDConstants.rotI);
    SmartDashboard.putNumber("Rot D", PIDConstants.rotD);

    configureBindings();
  }

  private void configureBindings() {
    commandController.a().onTrue(new RotatePIDCmd(driveSub,
     SmartDashboard.getNumber("Rot P", PIDConstants.rotP),
     SmartDashboard.getNumber("Rot I", PIDConstants.rotI),
     SmartDashboard.getNumber("Rot D", PIDConstants.rotD),
     0
    ));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
