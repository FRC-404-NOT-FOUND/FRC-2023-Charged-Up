// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.subArm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class A_Pivot extends SubsystemBase {
  public final CANSparkMax pivotMotor = new CANSparkMax(Constants.PIVOT_MOTOR_INDEX, MotorType.kBrushless);
  RelativeEncoder encoder = pivotMotor.getEncoder();
  SparkMaxPIDController pid = pivotMotor.getPIDController();

  /** Creates a new A_Rotation. */
  public A_Pivot() {}

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
}
