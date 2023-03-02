// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IMU;

public class TryReconnectArduino extends CommandBase {
  private IMU s_imu;

  public TryReconnectArduino(IMU imu) {
    addRequirements(imu);
    s_imu = imu;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!s_imu.isArduinoConnected()){
      s_imu.connectArduino();
    } else {
      System.out.println("Arduino is already connected dumbo.");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
