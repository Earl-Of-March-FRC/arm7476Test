// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  WPI_TalonSRX rightLift = new WPI_TalonSRX(6);
  WPI_TalonSRX leftLift = new WPI_TalonSRX(0);
  WPI_VictorSPX extend = new WPI_VictorSPX(9);

  public ArmSubsystem() {
    rightLift.setNeutralMode(NeutralMode.Brake);
    leftLift.setNeutralMode(NeutralMode.Brake);
    extend.setNeutralMode(NeutralMode.Brake);
    rightLift.setInverted(false);
    leftLift.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
  }

  public void setRightLiftSpeed(double speed){
    rightLift.set(speed);
  }
  public void setLeftLiftSpeed(double speed){
    leftLift.set(speed);
  }
  public void setExtendSpeed(double speed){
    extend.set(speed);
  }

  public void lift(double speed){
    rightLift.set(speed);
    leftLift.set(speed);
  }
}
