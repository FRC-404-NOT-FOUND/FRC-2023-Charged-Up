// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class MecanumDrive extends CommandBase {
  Drivetrain drivetrain;
  Supplier<Double> horizontalFunction;
  Supplier<Double> verticalFunction;
  Supplier<Double> pivotFunction;

  public MecanumDrive(Drivetrain d, Supplier<Double> h, Supplier<Double> v, Supplier<Double> p) {
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
    // System.out.println("Horiz:" + horizontalFunction.get());
    // System.out.println("Vert:" + verticalFunction.get());
    // System.out.println("Pivot:" + pivotFunction.get());

    double horizontal = horizontalFunction.get();
    double vertical = verticalFunction.get();
    double pivot = pivotFunction.get();

    drivetrain.driveCartesian(-vertical, horizontal, pivot);

    // double horizontalSign = horizontalFunction.get()/Math.abs(horizontalFunction.get());
    // double verticalSign = verticalFunction.get() / Math.abs(verticalFunction.get());
    // double pivotSign = pivotFunction.get() / Math.abs(pivotFunction.get());

    // double horizontal = Math.pow(horizontalFunction.get(), 2);
    // double vertical = Math.pow(verticalFunction.get(), 2);
    // double pivot = Math.pow(pivotFunction.get(), 2);

    // drivetrain.driveCartesian(-(vertical * verticalSign), horizontal * horizontalSign, pivot * pivotSign);
    // drivetrain.driveCartesian(vertical, horizontal, pivot);
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
