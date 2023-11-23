// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
      //Motors
      final WPI_VictorSPX extensionMotor = OperatorConstants.extension;
      final WPI_TalonSRX rightLift = OperatorConstants.rightLift;
      final WPI_TalonSRX leftLift = OperatorConstants.leftLift;
  public ArmSubsystem() {


    //Encoders
    extensionMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    extensionMotor.setSelectedSensorPosition(0);
    extensionMotor.setSensorPhase(true);
    
    rightLift.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    rightLift.setSelectedSensorPosition(0);
    rightLift.setSensorPhase(true);

    leftLift.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    leftLift.setSelectedSensorPosition(0);
    leftLift.setSensorPhase(true);

    extensionMotor.setNeutralMode(NeutralMode.Brake);
    rightLift.setNeutralMode(NeutralMode.Brake);
    leftLift.setNeutralMode(NeutralMode.Brake);
  }


  public double getPositionExtension(){
    return extensionMotor.getSelectedSensorPosition();
  }

  public double getPositionRightLift(){
    return rightLift.getSelectedSensorPosition();
  }

  public double getPositionLeftLift(){
    return leftLift.getSelectedSensorPosition();
  }

  public void setExtensionSpeed(double input){
    extensionMotor.set(input);
  }

  public void setLiftSpeed(double input){
    rightLift.set(input);
    leftLift.set(input);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
