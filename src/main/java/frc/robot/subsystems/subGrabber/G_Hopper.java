// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.subGrabber;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class G_Hopper extends SubsystemBase {
  public Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  public DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0); //CONSTANT THESE
  /** Creates a new G_Hopper. */
  public G_Hopper() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
