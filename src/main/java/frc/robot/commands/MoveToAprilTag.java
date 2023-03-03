// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class MoveToAprilTag extends CommandBase {
  private final Drivetrain drivetrain;
  private final PIDCommand apx;
  private final PIDCommand apy;

  /** Creates a new MoveToAprilTag. */
  public MoveToAprilTag(Drivetrain d) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(d);
    
    drivetrain = d;
    apx = new MoveToAprilTagX();
    apy = new MoveToAprilTagY();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Attempting Approach");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    apx.schedule();
    apy.schedule();

    drivetrain.driveCartesian(Constants.aprilTagMoveHorizontal, Constants.aprilTagMoveVertical, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Constants.aprilTagMoveHorizontal = 0;
    Constants.aprilTagMoveVertical = 0;
    drivetrain.driveCartesian(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
