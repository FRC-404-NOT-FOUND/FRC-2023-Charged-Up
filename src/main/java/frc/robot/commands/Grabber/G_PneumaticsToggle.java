// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Grabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.Grabber;

public class G_PneumaticsToggle extends CommandBase {
  Grabber s_grabber;
  public G_PneumaticsToggle(Grabber grabber) {
    addRequirements(grabber);
    s_grabber = grabber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_grabber.pneumaticsClose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_grabber.pneumaticsOpen();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
