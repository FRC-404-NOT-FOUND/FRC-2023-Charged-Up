// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class A_Disengage extends CommandBase {
  
  //One-Shot Command
  //Retracts Arm to 0.
  //Opens Pnumatics (If they aren't already)
  //Swings arm back to 0 Degrees (SLOWLY)

  Arm s_arm;
  A_extendTo extendTo;

  public A_Disengage(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);

    s_arm = arm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
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
