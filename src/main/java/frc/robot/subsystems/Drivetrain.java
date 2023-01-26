// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private Spark frontLeftMotor = new Spark(Constants.FRONT_LEFT_MOTOR_INDEX);
  private Spark frontRightMotor = new Spark(Constants.FRONT_RIGHT_MOTOR_INDEX);
  private Spark backLeftMotor = new Spark(Constants.BACK_LEFT_MOTOR_INDEX);
  private Spark backRightMotor = new Spark(Constants.BACK_RIGHT_MOTOR_INDEX);
  
  public Drivetrain() {}

  //Setters
  public void setFrontLeftMotor(double speed) {this.frontLeftMotor.set(speed);}
  public void setFrontRightMotor(double speed) {this.frontRightMotor.set(speed);}
  public void setBackLeftMotor(double speed) {this.backLeftMotor.set(speed);}
  public void setBackRightMotor(double speed) {this.backRightMotor.set(speed);}

  public void setInvertfrontLeftMotor(boolean isInverted){this.frontLeftMotor.setInverted(isInverted);}
  public void setInvertfrontRightMotor(boolean isInverted){this.frontRightMotor.setInverted(isInverted);}
  public void setInvertBackLeftMotor(boolean isInverted){this.backLeftMotor.setInverted(isInverted);}
  public void setInvertBackRightMotor(boolean isInverted){this.backRightMotor.setInverted(isInverted);}
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void setDefaultCommand(Command defaultCommand) {
      super.setDefaultCommand(defaultCommand);
  }
}
