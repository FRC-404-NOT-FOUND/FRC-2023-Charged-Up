// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.subArm.A_Extension;
import frc.robot.subsystems.subArm.A_Pivot;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */

  private A_Extension extension;
  private A_Pivot pivot;

  public Arm() {
    extension = new A_Extension();
    pivot = new A_Pivot();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public A_Extension getExtension(){
    return extension;
  }

  public A_Pivot getPivot(){
    return pivot;
  }

  public double getExtensionEncoderPosition(){
    return extension.getEncoderPosition();
  }
  public double getExtensionEncoderVelocity(){
    return extension.getEncoderVelocity();
  }
  public double getPivotEncoderPosition(){
    return pivot.getEncoderPosition();
  }
  public double getPivotEncoderVelocity(){
    return pivot.getEncoderVelocity();
  }

  public Command extendContinuousCommand() {
    return Commands.startEnd(
      () -> getExtension().setMotorWheel(0.5),
      () -> getExtension().setMotorWheel(0),
      getExtension()
    );
  }

  public Command retractContinuousCommand() {
    return Commands.startEnd(
      () -> getExtension().setMotorWheel(-0.5),
      () -> getExtension().setMotorWheel(0),
      getExtension()
    );
  }

  public Command raiseContinuousCommand(DoubleSupplier d) {
    return Commands.startEnd(
      () -> getPivot().setMotorWheel(d.getAsDouble()),
      () -> getPivot().setMotorWheel(0),
      getPivot()
    );
  }

  public Command lowerContinuousCommand(DoubleSupplier d) {
    return Commands.startEnd(
      () -> getPivot().setMotorWheel(-d.getAsDouble()),
      () -> getPivot().setMotorWheel(0),
      getPivot()
    );
  }

  public Command extendArmTo(double d) {
    return Commands.either(retractContinuousCommand(), extendContinuousCommand(), () -> d < getExtensionEncoderPosition())
      .until(() -> getExtensionEncoderPosition() + 3 > d && getExtensionEncoderPosition() - 3 < d)
      .finallyDo((interrupted) -> getExtension().getPIDController().setReference(d, ControlType.kPosition));
  }

  public Command pivotArmTo(double d) {
    return Commands.runOnce(
      () -> getPivot().getPIDController().setReference(d, ControlType.kPosition),
      getPivot()
    );
  }

  public Command moveArmTo(double angle, double ext) {
    return Commands.sequence(pivotArmTo(angle), extendArmTo(ext));
  }

  public Command moveToDefault() {
    return Commands.sequence(extendArmTo(Constants.DEFAULT_EXTENSION), pivotArmTo(Constants.DEFAULT_ANGLE));
  }

  public Command engage() {
    return Commands.sequence(pivotArmTo(Constants.DEFAULT_ANGLE), extendArmTo(Constants.DEFAULT_EXTENSION));
  }
}
