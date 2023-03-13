// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.subGrabber;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class G_Hopper extends SubsystemBase {
  private Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  private DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 
                Constants.HOPPER_PNEUMATIC_FWD_PORT, Constants.HOPPER_PNEUMATIC_REV_PORT); 

  private boolean isPneumaticsClosed = false;
  
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

  public void turnCompressorOn(){
    compressor.enableDigital();
    System.out.println("Compressor On!");
  }

  public void turnCompressorOff(){
    compressor.disable();
    System.out.println("Compressor Off!");
  }

  public void pneumaticsOpen(){
    doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    isPneumaticsClosed = false;
    System.out.println("Pneumatic Grabber Opened");
  }

  public void pneumaticsClose(){
    doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    isPneumaticsClosed = true;
    System.out.println("Pneumatic Grabber Closed");
  }

  //Returns true if the pneumatics system is closed
  public boolean pneumaticsCloseState(){
    return isPneumaticsClosed;
  }

  //Returns true if the pneumatics system is open
  public boolean pneumaticsOpenState(){
    return !isPneumaticsClosed;
  }

  public Command toggleCompressorCommand() {
    return Commands.startEnd(this::turnCompressorOn, this::turnCompressorOff);
  }

  public Command toggleGrabberCommand() {
    return startEnd(this::pneumaticsClose, this::pneumaticsOpen);
  }
}
