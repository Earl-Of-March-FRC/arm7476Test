// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.Supplier;

public class ArmFeedFoward extends CommandBase {
  /** Creates a new ArmFeedFoward. */
  private ArmSubsystem armsub;
  private Supplier<Double> liftSpeed;
  private Supplier<Double> extendSpeed;

  public ArmFeedFoward(ArmSubsystem armsub, Supplier<Double> liftSpeed ,Supplier<Double> extendSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armsub = armsub;
    this.liftSpeed = liftSpeed;
    this.extendSpeed = extendSpeed;
    addRequirements(armsub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armsub.setExtendSpeed(extendSpeed.get());
    armsub.lift(liftSpeed.get());
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
