// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.subsystems.Drivetrain;

public class MoveHorizontal extends CommandBase {
  Drivetrain drivetrain;
  double speed;
  boolean shouldEnd = false;

  /** Creates a new MoveHorizontal. */
  public MoveHorizontal(Drivetrain d, double s) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(d);

    drivetrain = d;
    speed = s;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Limelight.setRetroflective();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.driveCartesian(0, speed, 0);
    if (Limelight.isValidTarget()) {
      shouldEnd = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.driveCartesian(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shouldEnd;
  }
}
