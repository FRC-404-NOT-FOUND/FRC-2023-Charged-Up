// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmRaiseContinuous extends CommandBase {
  private final Arm arm;
  private final DoubleSupplier ds;
  /** Creates a new ArmRaiseContinuous. */
  public ArmRaiseContinuous(Arm a, DoubleSupplier d) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(a.getPivot());
    arm = a;
    ds = d;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.getPivot().setMotorWheel(ds.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.getPivot().setMotorWheel(0);
    arm.getPivot().getPIDController().setReference(arm.getPivotEncoderPosition(), ControlType.kPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
