// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.subArm.A_Extension;
import frc.robot.subsystems.subArm.A_Pivot;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */

  private A_Extension extension;
  private A_Pivot pivot;

  public Arm() {
    extension = new A_Extension();
    pivot = new A_Pivot();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public A_Extension getExtension(){
    return extension;
  }

  public A_Pivot getPivot(){
    return pivot;
  }

  public double getExtensionEncoderPosition(){
    return extension.getEncoderPosition();
  }
  public double getExtensionEncoderVelocity(){
    return extension.getEncoderVelocity();
  }
  public double getPivotEncoderPosition(){
    return pivot.getEncoderPosition();
  }
  public double getPivotEncoderVelocity(){
    return pivot.getEncoderVelocity();
  }
}
