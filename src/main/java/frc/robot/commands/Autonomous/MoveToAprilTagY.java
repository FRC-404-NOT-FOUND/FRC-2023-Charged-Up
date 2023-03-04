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
//Y = Side to Side movement.
public class MoveToAprilTagY extends PIDCommand {
  public MoveToAprilTagY() {
    super(
      // The controller that the command will use
      new PIDController(
          Constants.DRIVETRAIN_TRANSFORM_KPy, 
          Constants.DRIVETRAIN_TRANSFORM_KIy, 
          Constants.DRIVETRAIN_TRANSFORM_KDy
        ),
      // This should return the measurement
      () -> Limelight.getTableEntry("botpose_targetspace").getDoubleArray(new double[6])[0],
      // This should return the setpoint (can also be a constant)
      () -> 0.0,
      // This uses the output
      output -> {
        Constants.aprilTagMoveY = output;
      });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.

    getController().enableContinuousInput(-10, 10);
    getController().setTolerance(0.1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
