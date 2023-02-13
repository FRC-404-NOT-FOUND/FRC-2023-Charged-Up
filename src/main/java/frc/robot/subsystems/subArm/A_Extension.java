// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.subArm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class A_Extension extends SubsystemBase {
  private final CANSparkMax motorWheel = new CANSparkMax(Constants.EXTENSION_WHEEL_MOTOR_INDEX, MotorType.kBrushless);
  private RelativeEncoder encoder = motorWheel.getEncoder();
  private SparkMaxPIDController pid = motorWheel.getPIDController();
  /** Creates a new A_Extension. */
  public A_Extension() {
    pid.setFeedbackDevice(encoder);
    pid.setOutputRange(Constants.EXTENSION_WHEEL_MIN_POSITION, Constants.EXTENSION_WHEEL_MAX_POSITION);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public double getEncoderPosition(){
    return encoder.getPosition();
  }
  public double getEncoderVelocity(){
    return encoder.getVelocity();
  }

  public SparkMaxPIDController getPIDController(){
    return pid;
  }
  public void setMotorWheel(double speed){
    motorWheel.set(speed);
  }
}
