// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class A_extendContinuous extends CommandBase {
  private final Arm arm;
  private final double multiplier;

  /** Creates a new A_extendContinuous. */
  public A_extendContinuous(Arm a, double m) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(a);

    arm = a;
    multiplier = m;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.getExtension().setMotorWheel(0.2 * multiplier);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.getExtension().setMotorWheel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
