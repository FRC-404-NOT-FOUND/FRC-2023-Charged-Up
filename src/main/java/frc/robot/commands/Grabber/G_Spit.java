// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Grabber;

public class G_Spit extends CommandBase {
  private final Grabber grabber;

  /** Creates a new G_Blow. */
  public G_Spit(Grabber g) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(g);

    grabber = g;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("DA SPIT");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    grabber.startSpit();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    grabber.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}