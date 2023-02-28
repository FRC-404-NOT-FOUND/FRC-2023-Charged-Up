// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class MecanumDriveWithMotorOffsets extends CommandBase {
  private final Drivetrain drivetrain;
  DoubleSupplier horizontalFunction;
  DoubleSupplier verticalFunction;
  DoubleSupplier pivotFunction;

  public MecanumDriveWithMotorOffsets(Drivetrain d, DoubleSupplier h, DoubleSupplier v, DoubleSupplier p) {
    drivetrain = d;
    horizontalFunction = h;
    verticalFunction = v;
    pivotFunction = p;

    addRequirements(drivetrain);
  }
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double horizontal = horizontalFunction.getAsDouble();
    double vertical = verticalFunction.getAsDouble();
    double pivot = pivotFunction.getAsDouble();

    MecanumDriveWheelSpeeds wheelSpeeds = drivetrain.driveCartesianIK(-vertical / 2, horizontal / 2, pivot / 2);
    wheelSpeeds.frontLeftMetersPerSecond *= Constants.FRONT_LEFT_MOTOR_SPEED_OFFSET;
    wheelSpeeds.frontRightMetersPerSecond *= Constants.FRONT_RIGHT_MOTOR_SPEED_OFFSET;
    wheelSpeeds.rearLeftMetersPerSecond *= Constants.BACK_LEFT_MOTOR_SPEED_OFFSET;
    wheelSpeeds.rearRightMetersPerSecond *= Constants.BACK_RIGHT_MOTOR_SPEED_OFFSET;

    if (Math.abs(horizontal) >= Constants.GAMEPAD_DEADZONE || Math.abs(vertical) >= Constants.GAMEPAD_DEADZONE
        || Math.abs(pivot) >= Constants.GAMEPAD_DEADZONE) {
      drivetrain.setWheelSpeeds(wheelSpeeds);
    } else {
      drivetrain.setWheelSpeeds(new MecanumDriveWheelSpeeds(0, 0, 0, 0));
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
