// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.subArm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class A_Pivot extends SubsystemBase {
  public final CANSparkMax pivotMotor = new CANSparkMax(Constants.PIVOT_MOTOR_INDEX, MotorType.kBrushless);
  //CREATE ENCODER
  
  /** Creates a new A_Pivot. */
  public A_Pivot() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
