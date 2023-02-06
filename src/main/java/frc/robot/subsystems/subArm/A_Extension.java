// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.subArm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class A_Extension extends SubsystemBase {
  private final CANSparkMax motorWheel = new CANSparkMax(Constants.EXTENSION_WHEEL_MOTOR_INDEX, MotorType.kBrushless);

  /** Creates a new A_Extension. */
  public A_Extension() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
