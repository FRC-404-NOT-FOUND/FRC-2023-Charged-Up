// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.subGrabber;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class G_Hopper extends SubsystemBase {
  private Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  private DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
                Constants.HOPPER_PNEUMATIC_FWD_PORT, Constants.HOPPER_PNEUMATIC_REV_PORT); 
  
  //private boolean enabled = false;
  public G_Hopper() {}

  @Override
  public void periodic() {}

  /*
  public void toggleCompressor() {
    enabled = !enabled;
    if (enabled) {
      compressor.enableDigital();
    } else {
      compressor.disable();
    }
  }
  */

  public Compressor getCompressor(){
    return compressor;
  }

  public DoubleSolenoid getDoubleSolenoid(){
    return doubleSolenoid;
  }
}
