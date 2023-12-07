// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class DrivetrainSubsystem extends SubsystemBase {
  private final WPI_TalonFX frontLeft = new WPI_TalonFX(DrivetrainConstants.frontLeftID);
  private final WPI_TalonFX frontRight = new WPI_TalonFX(DrivetrainConstants.frontRightID);
  private final WPI_TalonFX backLeft = new WPI_TalonFX(DrivetrainConstants.backLeftID);
  private final WPI_TalonFX backRight = new WPI_TalonFX(DrivetrainConstants.backRightID);
  private final TalonFXSimCollection frontLeftSim = new TalonFXSimCollection(frontLeft);
  private final TalonFXSimCollection frontRightSim = new TalonFXSimCollection(frontRight);
  private final TalonFXSimCollection backLeftSim = new TalonFXSimCollection(backLeft);
  private final TalonFXSimCollection backRightSim = new TalonFXSimCollection(backRight);

  private final AHRS gyro = new AHRS(Port.kUSB);

  private final MecanumDrive drivetrain = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);

  private double sX = 0;
  private double sY = 0;
  private double rX = 0;

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    frontLeft.setNeutralMode(NeutralMode.Brake);
    frontRight.setNeutralMode(NeutralMode.Brake);
    backLeft.setNeutralMode(NeutralMode.Brake);
    backRight.setNeutralMode(NeutralMode.Brake);

    frontLeft.setInverted(true);
    backLeft.setInverted(true);

    resetGyro();
    frontLeft.setSelectedSensorPosition(0);
    frontRight.setSelectedSensorPosition(0);
    backLeft.setSelectedSensorPosition(0);
    backRight.setSelectedSensorPosition(0);
    // gyro.calibrate();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("FL Speed", frontLeft.get());
    SmartDashboard.putNumber("FR Speed", frontRight.get());
    SmartDashboard.putNumber("BL Speed", backLeft.get());
    SmartDashboard.putNumber("BR Speed", backRight.get());

    SmartDashboard.putNumber("FL Distance", getFrontLeftDistance());
    SmartDashboard.putNumber("FR Distance", getFrontRightDistance());
    SmartDashboard.putNumber("BL Distance", getBackLeftDistance());
    SmartDashboard.putNumber("BR Distance", getBackRightDistance());

    SmartDashboard.putNumber("SX", sX);
    SmartDashboard.putNumber("SY", sY);
    SmartDashboard.putNumber("RX", rX);
   
    SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
  }

  @Override
  public void simulationPeriodic() {
    frontLeftSim.addIntegratedSensorPosition((int)(frontLeft.get() * 2048 * 0.02));
    backLeftSim.addIntegratedSensorPosition((int)(backLeft.get() * 2048 * 0.02));
    frontRightSim.addIntegratedSensorPosition((int)(frontRight.get() * 2048 * 0.02));
    backRightSim.addIntegratedSensorPosition((int)(backRight.get() * 2048 * 0.02));
  }

  public void setPermanent(double xSpeed, double ySpeed, double xRot) {
    drivetrain.driveCartesian(xSpeed, ySpeed, xRot);
    logMecanum(xSpeed, ySpeed, xRot);
  }

  public void set(double xSpeed, double ySpeed, double xRot) {
    //if (!gyroCalibrating()) {
    drivetrain.driveCartesian(xSpeed, ySpeed, xRot, getGyroRot2D().times(-1));
    logMecanum(xSpeed, ySpeed, xRot);
    //}
  }

  public Double getFrontLeftDistance() {
    return frontLeft.getSelectedSensorPosition();
  }
  public Double getBackLeftDistance() {
    return backLeft.getSelectedSensorPosition();
  }
  public Double getFrontRightDistance() {
    return frontRight.getSelectedSensorPosition();
  }
  public Double getBackRightDistance() {
    return backRight.getSelectedSensorPosition();
  }
  public double distanceToTicks(double distanceInches) {
    return (distanceInches / 6 * Math.PI) * 2048;
  }

  public Rotation2d getGyroRot2D() {
    return gyro.getRotation2d();
  }
  public double getGyroAngle() {
    return gyro.getAngle();
  }
  public double getGyroPID() {
    return (getGyroAngle() % 360);
  }
  public void resetGyro() {
    gyro.reset();
  }

  public void logMecanum(double sx, double sy, double rx) {
    this.sX = sx;
    this.sY = sy;
    this.rX = rx;
  }
}
