// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

//X = Forward
public class MoveToAprilTagX extends PIDCommand {
  

  public MoveToAprilTagX() {
    super(
        // The controller that the command will use
        new PIDController(
            Constants.DRIVETRAIN_TRANSFORM_KPx, 
            Constants.DRIVETRAIN_TRANSFORM_KIx, 
            Constants.DRIVETRAIN_TRANSFORM_KDx
        ),
        // This should return the measurement
        () -> Limelight.getTableEntry("botpose_targetspace").getDoubleArray(new double[6])[2],
        // This should return the setpoint (can also be a constant)
        () -> 0.83,
        // This uses the output
        output -> {
          Constants.aprilTagMoveX = output;
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    
    getController().enableContinuousInput(0, 100);
    getController().setTolerance(0.2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
