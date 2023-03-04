// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.subIMU.deprecated;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gyroscope extends SubsystemBase {
  private final ADXRS450_Gyro gyro;

  /** Creates a new Gyroscope. */
  public Gyroscope() {
    gyro = new ADXRS450_Gyro(Constants.ADXRS450_GYRO_PORT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void calibrate() {
    gyro.calibrate();
  }

  public double getYaw() {
    if (gyro.isConnected()) {
      return gyro.getAngle();
    }

    return 0;
  }
}
