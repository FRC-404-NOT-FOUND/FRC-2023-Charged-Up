// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.drive.MecanumDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IMU;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DrivetrainBalancePID extends ProfiledPIDCommand {
  //ROBOT MUST BE FACING FORWARD, ARM TOWARDS THE BALANCE
  //Only takes the pitch, and gets the 
  public DrivetrainBalancePID(Drivetrain drivetrain, IMU imu) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            Constants.DRIVETRAIN_TRANSFORM_KPx,
            Constants.DRIVETRAIN_TRANSFORM_KIx,
            Constants.DRIVETRAIN_TRANSFORM_KDx,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(Constants.MAX_AUTONOMOUS_WHEEL_SPEED, Constants.MAX_AUTONOMOUS_WHEEL_ACCEL)),
        // This should return the measurement
        () -> imu.getGyroPitch(),
        // This should return the goal (can also be a constant)
        // Tl;dr THIS IS THE TARGET, and the pitch should be zero (or close to it.).
        () -> new TrapezoidProfile.State(),
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          drivetrain.setWheelSpeeds(new WheelSpeeds(output, output, output, output));
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(drivetrain);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
