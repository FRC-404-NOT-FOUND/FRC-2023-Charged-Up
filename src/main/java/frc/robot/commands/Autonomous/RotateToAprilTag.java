// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateToAprilTag extends PIDCommand {
  public RotateToAprilTag() {
    super(
      // The controller that the command will use
      new PIDController(
          Constants.DRIVETRAIN_ROTATE_KP, 
          Constants.DRIVETRAIN_ROTATE_KI, 
          Constants.DRIVETRAIN_ROTATE_KD
        ),
      // This should return the measurement
      () -> Limelight.getTableEntry("tx").getDouble(0.0),
      // This should return the setpoint (can also be a constant)
      () -> 0.0,
      // This uses the output
      output -> {
        Constants.aprilTagRotate = -output;
      });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().enableContinuousInput(-29.8, 29.8);
    getController().setTolerance(0.05);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
