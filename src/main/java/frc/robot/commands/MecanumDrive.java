// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class MecanumDrive extends CommandBase {
  Drivetrain drivetrain;
  DoubleSupplier horizontalFunction;
  DoubleSupplier verticalFunction;
  DoubleSupplier pivotFunction;

  public MecanumDrive(Drivetrain d, DoubleSupplier h, DoubleSupplier v, DoubleSupplier p) {
    drivetrain = d;
    horizontalFunction = h;
    verticalFunction = v;
    pivotFunction = p;

    addRequirements(d);
  }
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double horizontal = Math.pow(horizontalFunction.getAsDouble(), 2);
    double vertical = Math.pow(verticalFunction.getAsDouble(), 2);
    double pivot = Math.pow(pivotFunction.getAsDouble(), 2);

    if (Math.abs(horizontal) >= Constants.GAMEPAD_DEADZONE || Math.abs(vertical) >= Constants.GAMEPAD_DEADZONE
        || Math.abs(pivot) >= Constants.GAMEPAD_DEADZONE) {
      drivetrain.driveCartesian(-vertical, horizontal, pivot);
    } else {
      drivetrain.driveCartesian(0, 0, 0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.driveCartesian(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
