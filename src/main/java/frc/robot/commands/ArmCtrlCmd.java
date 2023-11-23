// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ArmSubsystem;


public class ArmCtrlCmd extends CommandBase {
  private ArmSubsystem armSub;
  private Supplier<Double> extensionSpeed, rightliftSpeed, leftLiftSpeed;

  public ArmCtrlCmd(ArmSubsystem armSub, Supplier<Double> extensionSpeed, Supplier<Double> rightliftSpeed, Supplier<Double> leftLiftSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSub = armSub;
    this.extensionSpeed = extensionSpeed;
    this.rightliftSpeed = rightliftSpeed;
    this.leftLiftSpeed = leftLiftSpeed;
    addRequirements(armSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSub.setExtensionSpeed(extensionSpeed.get());
    armSub.setLiftSpeed(rightliftSpeed.get());
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
