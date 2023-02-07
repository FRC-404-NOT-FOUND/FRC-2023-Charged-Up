// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import frc.robot.subsystems.Drivetrain;

import frc.robot.Constants;

public class DriveAlongPath extends CommandBase {
  //https://github.wpilib.org/allwpilib/docs/beta/java/edu/wpi/first/wpilibj2/command/MecanumControllerCommand.html
  MecanumControllerCommand mcmCtrCmd;
  

  /** Creates a new DriveAlongPath. */

  public DriveAlongPath(Drivetrain drivetrain, Trajectory trajectory) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    mcmCtrCmd = new MecanumControllerCommand(
      trajectory, 
      null,                                  //PUT THE POSE SUPPLIER HERE! Uses the current pose.
      null,                           //TBD, I'm still working out what this is.
      drivetrain.getMecanumDriveKinematics(),     //Gets the chassis + measurements of the robot.
      null,                           //TrajectoryTracker PID Controller for the X movement of the robot
      null,                           //TrajectoryTracker PID Controller for the Y movement of the robot
      null,                       //TrajectoryTracker PID Controller for the Theta angle of the robot
      null,                       //Desired rotation at the end
      Constants.MAX_AUTONOMOUS_WHEEL_SPEED,       //The max speed the wheels can travel at
      null,                   //PID controller for the voltage of the Front Left wheel
      null,                    //PID controller for the voltage of the Rear Left wheel
      null,                  //PID controller for the voltage of the Front Right wheel
      null,                   //PID controller for the voltage of the Rear Right wheel
      ()-> drivetrain.getWheelSpeeds(),            //Supplier for the MecanumDriveWheelspeeds.
      null,                   //Consumer for the output MecanumDriveMotorVoltages
      drivetrain                                   //The required Subsystem(s)
      );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mcmCtrCmd.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
