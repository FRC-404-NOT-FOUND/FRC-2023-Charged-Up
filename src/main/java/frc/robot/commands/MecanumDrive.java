// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class MecanumDrive extends CommandBase {
  Drivetrain drivetrain;
  Supplier<Double> horizontalFunction;
  Supplier<Double> verticalFunction;
  Supplier<Double> pivotFunction;
  boolean isSquared;

  public MecanumDrive(Drivetrain d, Supplier<Double> h, Supplier<Double> v, Supplier<Double> p, boolean squared) {
    drivetrain = d;
    horizontalFunction = h;
    verticalFunction = v;
    pivotFunction = p;
    isSquared = squared;

    addRequirements(d);
  }
  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double h = horizontalFunction.get();
    double v = verticalFunction.get();
    double z = pivotFunction.get();

    // System.out.println("Horiz:" + h);
    // System.out.println("Vert:" + v);
    // System.out.println("Pivot:" + z);

    double horizontalSign = h / Math.abs(h);
    double verticalSign = v / Math.abs(v);
    // double pivotSign = pivotFunction.get() / Math.abs(pivotFunction.get());

    double horizontal = h * h;
    double vertical = v * v;
    double zRotation = z * 0.5;

    if(!isSquared){
      drivetrain.driveCartesian(-v, -h, -zRotation);
    }
    else{
      drivetrain.driveCartesian(-(vertical * verticalSign), -(horizontal * horizontalSign), -zRotation);
    }
    
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
