// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class MoveToAprilTag extends CommandBase {
  private final Drivetrain drivetrain;
  private final ParallelCommandGroup pid;

  /** Creates a new MoveToAprilTag. */
  public MoveToAprilTag(Drivetrain d) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(d);
    
    drivetrain = d;

    pid = new ParallelCommandGroup(
      new MoveToAprilTagX(), 
      new MoveToAprilTagY(), 
      new RotateToAprilTag()
    );

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Attempting Approach");

    pid.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pid.execute();

    drivetrain.driveCartesian(-Constants.aprilTagMoveY, Constants.aprilTagMoveX, Constants.aprilTagRotate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pid.end(interrupted);
    Constants.aprilTagMoveX = 0;
    Constants.aprilTagMoveY = 0;
    Constants.aprilTagRotate = 0;
    drivetrain.driveCartesian(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.isFinished();
  }
}
