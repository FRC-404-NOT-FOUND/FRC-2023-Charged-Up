// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class A_extendTo extends CommandBase {
  /** Creates a new A_extendTo. */
  private Arm arm;
  private double position;

  public A_extendTo(double p, Arm a) {
    addRequirements(a);
    // Use addRequirements() here to declare subsystem dependencies.
    arm = a;
    position = p;
  }

  @Override
  public void initialize(){
  }

  @Override
  public void execute() {
    arm.getExtension().getPIDController().setReference(position, CANSparkMax.ControlType.kPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
