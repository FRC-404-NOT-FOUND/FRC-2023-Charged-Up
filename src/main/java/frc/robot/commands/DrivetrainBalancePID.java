// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DrivetrainBalancePID extends ProfiledPIDCommand {
  //ROBOT MUST BE FACING FORWARD, ARM TOWARDS THE BALANCE
  public DrivetrainBalancePID(Drivetrain drivetrain) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            Constants.DRIVETRAIN_TRANSFORM_KP,
            Constants.DRIVETRAIN_TRANSFORM_KI,
            Constants.DRIVETRAIN_TRANSFORM_KD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(Constants.MAX_AUTONOMOUS_WHEEL_SPEED, Constants.MAX_AUTONOMOUS_WHEEL_ACCEL)),
        // This should return the measurement
        () -> drivetrain.getCurrentPose().getY(), //This might need to be changed to getX, or something equivalent to this using pitch from the gyro.
        // This should return the goal (can also be a constant)
        //THIS IS THE TARGET.
        () -> new TrapezoidProfile.State(),
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          drivetrain.setWheelSpeeds(new MecanumDriveWheelSpeeds(output, output, output, output));
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
