// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.subGrabber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class G_Intake extends SubsystemBase {
  private final CANSparkMax main = new CANSparkMax(Constants.INTAKE_LEFT_MOTOR_INDEX, MotorType.kBrushless);
  private final CANSparkMax follower = new CANSparkMax(Constants.INTAKE_RIGHT_MOTOR_INDEX, MotorType.kBrushless);

  private final LinearFilter currentFilter = LinearFilter.movingAverage(10);
  private double filteredCurrent;
  
  /** Creates a new G_Intake. */
  public G_Intake() {
    follower.follow(main, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    filteredCurrent = currentFilter.calculate(getCurrent());
  }

  public void start(double speed) {
    main.set(speed);
  }

  public void stop() {
    main.set(0);
  }

  public void spit(double speed) {
    main.set(-speed);
  }

  // Gets the current for the main motor
  public double getCurrent() {
    return main.getOutputCurrent();
  }

  public double getFilteredCurrent() {
    return filteredCurrent;
  }

  public Command intakeCommand(double speed) {
    return startEnd(() -> start(speed), this::stop);
  }

  public Command spitCommand(double speed) {
    return startEnd(() -> spit(speed), this::stop);
  }
}
