// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.subGrabber.G_Hopper;
import frc.robot.subsystems.subGrabber.G_Intake;

public class Grabber extends SubsystemBase {
  /** Creates a new Grabber. */
  private G_Hopper hopper;
  private G_Intake intake;

  private boolean isPneumaticsClosed = false;
  
  public Grabber() {
    hopper = new G_Hopper();
    intake = new G_Intake();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void turnCompressorOn(){
    hopper.getCompressor().enableDigital();
    System.out.println("Compressor On!");
  }

  public void turnCompressorOff(){
    hopper.getCompressor().disable();
    System.out.println("Compressor Off!");
  }

  public void pneumaticsOpen(){
    hopper.getDoubleSolenoid().set(DoubleSolenoid.Value.kForward);
    isPneumaticsClosed = false;
    System.out.println("Pneumatic Grabber Opened");
  }

  public void pneumaticsClose(){
    hopper.getDoubleSolenoid().set(DoubleSolenoid.Value.kReverse);
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

  public G_Hopper getHopper(){
    return hopper;
  }

  public G_Intake getIntake(){
    return intake;
  }

  public void startIntake() {
    intake.start();
  }

  public void stopIntake() {
    intake.stop();
  }

  public void startSpit() {
    intake.spit();
  }
}
