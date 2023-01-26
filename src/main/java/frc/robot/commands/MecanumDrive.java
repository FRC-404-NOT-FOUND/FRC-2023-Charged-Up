// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class MecanumDrive extends CommandBase {
  private final Drivetrain drivetrain;
  DoubleSupplier horizontalFunction;
  DoubleSupplier verticalFunction;
  DoubleSupplier pivotFunction;

  public MecanumDrive(Drivetrain d, DoubleSupplier h, DoubleSupplier v, DoubleSupplier p) {
    drivetrain = d;
    horizontalFunction = h;
    verticalFunction = v;
    pivotFunction = p;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double horizontal = horizontalFunction.getAsDouble() / 2;
    double vertical = verticalFunction.getAsDouble() / 2;
    double pivot = pivotFunction.getAsDouble() / 2;

    if(Math.abs(horizontal) >= Constants.DEADZONE || Math.abs(vertical) >= Constants.DEADZONE || Math.abs(pivot) >= Constants.DEADZONE){
      drivetrain.setFrontLeftMotor(-pivot + (vertical - horizontal));
      drivetrain.setBackLeftMotor(-pivot + (vertical + horizontal));
      drivetrain.setFrontRightMotor(pivot + (vertical + horizontal));
      drivetrain.setBackRightMotor(pivot + (vertical - horizontal));
    }
    else{
      drivetrain.setFrontLeftMotor(0);
      drivetrain.setBackLeftMotor(0);
      drivetrain.setFrontRightMotor(0);
      drivetrain.setBackRightMotor(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setFrontLeftMotor(0);
    drivetrain.setBackLeftMotor(0);
    drivetrain.setFrontRightMotor(0);
    drivetrain.setBackRightMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
