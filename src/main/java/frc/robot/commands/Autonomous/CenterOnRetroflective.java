// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Limelight;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CenterOnRetroflective extends PIDCommand {
  /** Creates a new CenterOnRetroflective. */
  public CenterOnRetroflective(Drivetrain d) {
    super(
        // The controller that the command will use
        new PIDController(Constants.DRIVETRAIN_TRANSFORM_KPx, Constants.DRIVETRAIN_TRANSFORM_KIx, Constants.DRIVETRAIN_TRANSFORM_KDx),
        // This should return the measurement
        () -> Limelight.getTableEntry("tx").getDouble(0),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          // Use the output here
          d.driveCartesian(0, output, 0);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(d);
    getController().enableContinuousInput(-30, 30);
    getController().setTolerance(0.2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
