// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.subIMU.deprecated;

import edu.wpi.first.wpilibj.ADXL362;
import edu.wpi.first.wpilibj.ADXL362.AllAxes;
import edu.wpi.first.wpilibj.ADXL362.Axes;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Accelerometer extends SubsystemBase {
  private final ADXL362 accel;

  /** Creates a new Accelerometer. */
  public Accelerometer() {
    accel = new ADXL362(Range.k4G);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public AllAxes getAccelerations() {
    return accel.getAccelerations();
  }

  public double getAcceleration(Axes axis) {
    return accel.getAcceleration(axis);
  }

  public double getX() {
    return accel.getX();
  }

  public double getY() {
    return accel.getY();
  }

  public double getZ() {
    return accel.getZ();
  }
}
