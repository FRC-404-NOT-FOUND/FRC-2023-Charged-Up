// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class FunnyAutonomous extends CommandBase {
  Drivetrain drivetrain;
  public FunnyAutonomous(Drivetrain d) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(d);
    drivetrain = d;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Auto Init");
    Constants.timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Constants.timer.get() < 0.25){
      drivetrain.driveCartesian(-0.5, 0.0, 0.0);
    } else if (Constants.timer.get() < 2.0){
      drivetrain.driveCartesian(0.0, 0.0, 0.0);
    } else if (Constants.timer.get() < 4.0){
      drivetrain.driveCartesian(0.5, 0.0, 0.0);
    } else {
      drivetrain.driveCartesian(0.0, 0.0, 0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Constants.timer.stop();
    Constants.timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
