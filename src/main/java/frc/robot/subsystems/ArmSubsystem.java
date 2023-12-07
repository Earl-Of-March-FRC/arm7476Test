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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
      //Motors
      final WPI_VictorSPX extensionMotor = OperatorConstants.extension;
      final WPI_TalonSRX rightLift = OperatorConstants.rightLift;
      final WPI_TalonSRX leftLift = OperatorConstants.leftLift;
      final Encoder encoder = new Encoder(0, 1);
  public ArmSubsystem() {

    rightLift.getSelectedSensorPosition(0);
    extensionMotor.setNeutralMode(NeutralMode.Brake);
    rightLift.setNeutralMode(NeutralMode.Brake);
    leftLift.setNeutralMode(NeutralMode.Brake);
  }


  public void setExtensionSpeed(double input){
    extensionMotor.set(input);
  }

  public void setLiftSpeed(double input){
    leftLift.set(input);
    rightLift.set(input);
  }

  public void setArmMotorSpeed(double inputLift, double inputExtension){
    // if(encoder.getRaw() >= 50 && inputLift < 0){
    //   setLiftSpeed(inputLift);
    //   setExtensionSpeed(inputExtension);
    // }
    // else if (encoder.getRaw() >= 50){
    //   setLiftSpeed(inputLift);
    //   setExtensionSpeed(inputExtension);
    // }
    // else if (encoder.getRaw() >= 20){
    //   setLiftSpeed(inputLift);
    //   setExtensionSpeed(inputExtension);
    // }
    // else if(encoder.getRaw() <= 5 && inputLift > 0){
    //   setLiftSpeed(inputLift);
    // }
    // else{
    //   setLiftSpeed(inputLift);
    // }
    setLiftSpeed(inputLift);
    setExtensionSpeed(inputExtension);

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Enocder ", rightLift.getSelectedSensorPosition());

    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
