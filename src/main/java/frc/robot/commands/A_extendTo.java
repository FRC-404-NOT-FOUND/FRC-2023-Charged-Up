// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.subArm.A_Extension;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class A_extendTo extends CommandBase {
  /** Creates a new A_extendTo. */
  private A_Extension extension;
  private double position;

  public A_extendTo(double p, A_Extension e) {
    addRequirements(e);
    // Use addRequirements() here to declare subsystem dependencies.
    extension = e;
    position = p;
  }

  @Override
  public void initialize(){
    extension.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
  }

  @Override
  public void execute() {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
