// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.subGrabber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class G_Intake extends SubsystemBase {
  
  private final CANSparkMax motorLeft = new CANSparkMax(0, MotorType.kBrushless); // INSERT CONSTANT
  private final CANSparkMax motorRight = new CANSparkMax(1, MotorType.kBrushless); // INSERT CONSTANT

  //CREATE ENCODER
  /** Creates a new G_Intake. */
  public G_Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
