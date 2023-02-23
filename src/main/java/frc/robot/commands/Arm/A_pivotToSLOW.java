// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class A_pivotToSLOW extends CommandBase {
  Arm s_arm;
  double angle;
  
  public A_pivotToSLOW(double a, Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);

    s_arm = arm;
    angle = a;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_arm.getPivot().getPIDController().setReference(angle, CANSparkMax.ControlType.kPosition, Constants.PIVOT_SLOW_PID_SLOT);
    //WE NEED A WAY TO DETERMINE FOR SURE WHEN IT ARRIVES AT THE ANGLE, IT STOPS RUNNING AND FINISHES THE COMMAND.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}