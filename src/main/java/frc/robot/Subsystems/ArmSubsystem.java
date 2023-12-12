// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private final WPI_VictorSPX extension = new WPI_VictorSPX(ArmConstants.extensionID);
  private final WPI_VictorSPX incline1 = new WPI_VictorSPX(ArmConstants.inclineID1);
  private final WPI_VictorSPX incline2 = new WPI_VictorSPX(ArmConstants.inclineID2);

  private final MotorControllerGroup incline = new MotorControllerGroup(incline1, incline2);

  private final Encoder encoder = new Encoder(0, 1);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    incline1.setNeutralMode(NeutralMode.Brake);
    incline2.setNeutralMode(NeutralMode.Brake);
    extension.setNeutralMode(NeutralMode.Brake);

    incline2.setInverted(true);

    incline2.setSelectedSensorPosition(ArmConstants.incline2DefaultPos);
  }

  public void setExtend(double speed) {
    extension.set(speed);
  }
  public void setIncline(double speed) {
    incline.set(speed);
  }
  public void stopExtend() {
    extension.stopMotor();
  }
  public void stopIncline() {
    incline.stopMotor();
  }

  public double getEncoderRate() {
    return encoder.getRate();
  }
  public double getInclineAngle() {
    return incline2.getSelectedSensorPosition() / (ArmConstants.incline2DefaultPos / ArmConstants.armMaxAngle);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Incline 1", incline1.getSelectedSensorPosition());
    SmartDashboard.putNumber("Incline Calculated Angle", getInclineAngle());
  }
}
