// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private VictorSP frontLeftMotor = new VictorSP(Constants.FRONT_LEFT_MOTOR_INDEX);
  private VictorSP frontRightMotor = new VictorSP(Constants.FRONT_RIGHT_MOTOR_INDEX);
  private VictorSP backLeftMotor = new VictorSP(Constants.BACK_LEFT_MOTOR_INDEX);
  private VictorSP backRightMotor = new VictorSP(Constants.BACK_RIGHT_MOTOR_INDEX);

  //ADD ENCODERS HERE!!!

  private MecanumDrive mDrive;

  public Drivetrain() {
    mDrive = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void setDefaultCommand(Command defaultCommand) {
    super.setDefaultCommand(defaultCommand);
  }

  public void driveCartesian(double xSpeed, double ySpeed, double zRotation) {
    mDrive.driveCartesian(xSpeed, ySpeed, zRotation);
  }

  public Supplier<MecanumDriveWheelPositions> getWheelPositions(){
    //Once we get encoders, we can put them to use here.
    return () -> new MecanumDriveWheelPositions(
      0, 
      0, 
      0, 
      0);
  }

  public Supplier<MecanumDriveWheelSpeeds> getWheelSpeeds(){
    //Once we get encoders, we can put them to use here.
    return () -> new MecanumDriveWheelSpeeds(
      0, 
      0, 
      0, 
      0);
  }

  // Setters
  public void setFrontLeftMotor(double speed) {
    this.frontLeftMotor.set(speed);
  }
  public void setFrontRightMotor(double speed) {
    this.frontRightMotor.set(speed);
  }
  public void setBackLeftMotor(double speed) {
    this.backLeftMotor.set(speed);
  }
  public void setBackRightMotor(double speed) {
    this.backRightMotor.set(speed);
  }

  public void setInvertfrontLeftMotor(boolean isInverted) {
    this.frontLeftMotor.setInverted(isInverted);
  }
  public void setInvertfrontRightMotor(boolean isInverted) {
    this.frontRightMotor.setInverted(isInverted);
  }
  public void setInvertBackLeftMotor(boolean isInverted) {
    this.backLeftMotor.setInverted(isInverted);
  }
  public void setInvertBackRightMotor(boolean isInverted) {
    this.backRightMotor.setInverted(isInverted);
  }

}
