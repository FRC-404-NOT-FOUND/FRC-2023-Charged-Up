// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.subArm.A_Extension;
import frc.robot.subsystems.subGrabber.G_Hopper;
import frc.robot.subsystems.subGrabber.G_Intake;

public class Grabber extends SubsystemBase {
  /** Creates a new Grabber. */
  private G_Hopper hopper;
  private G_Intake intake;
  
  public Grabber(G_Hopper h, G_Intake i) {
    hopper = h;
    intake = i;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
