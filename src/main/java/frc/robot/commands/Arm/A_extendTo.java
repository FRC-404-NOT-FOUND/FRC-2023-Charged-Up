// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class A_extendTo extends CommandBase {
  /** Creates a new A_extendTo. */
  private Arm arm;
  private double position;
  private boolean done = false;

  public A_extendTo(double p, Arm a) {
    //addRequirements(a.getExtension());
    // Use addRequirements() here to declare subsystem dependencies.
    arm = a;
    position = p;
  }

  @Override
  public void initialize(){
    System.out.println("Extendo to position: " + position);
  }

  @Override
  public void execute() {
    if(arm.getExtension().getEncoderPosition() >= position - 5
    && arm.getExtension().getEncoderPosition() <= position + 5){
      arm.getExtension().motorWheel.stopMotor();
      done = true;
    }
    else{
      if(arm.getPivot().getEncoderPosition() >= 250)
        arm.getExtension().getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
