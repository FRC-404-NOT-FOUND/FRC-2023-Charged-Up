// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import frc.robot.Constants;

public class DriveAlongPath extends CommandBase {
  //WPILIB version of MecanumControllerCommand
  //https://github.wpilib.org/allwpilib/docs/beta/java/edu/wpi/first/wpilibj2/command/MecanumControllerCommand.html
  //MecanumControllerCommand mcmCtrCmd;

  //Team 3015 Ranger Robotics's Path Planner MecanumControllerCommand
  //If we wanted to, we could use WPI-Lib's PathFollower... But do we really?
  //https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Java-Usage#ppmecanumcontrollercommand
  //Here's the pathplanner API: https://robotpy.readthedocs.io/projects/pathplannerlib/en/stable/api.html
  PPMecanumControllerCommand mcmCtrCmd;

  //NOTE: The PathPlanner also comes with a full auto generator. We should Implement that.
  //Leave this one though, it'll be good for driverside automation.
  //https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Java-Usage#autobuilder

  /** Creates a new DriveAlongPath. */
  public DriveAlongPath(PathPlannerTrajectory trajectory, Drivetrain drivetrain, boolean isFirstPath) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    if(isFirstPath){
      drivetrain.resetOdometry(
        trajectory.getInitialState().holonomicRotation,
        drivetrain.getWheelPositions(),
        trajectory.getInitialState().poseMeters
      );
    }

    mcmCtrCmd = new PPMecanumControllerCommand(
      trajectory,                               //PathPlannerTrajectory
      () -> drivetrain.getCurrentPose(),        //Pose Supplier (GETS THE CURRENT POSE EVERY TIME)
      drivetrain.getKinematics(),               //Kinematics of robot
                                                //X and Y PID COntrollers, using 0 for all uses feedforwards, TO BE TUNED in Constants
      new PIDController(Constants.DRIVETRAIN_TRANSFORM_KP, Constants.DRIVETRAIN_TRANSFORM_KI, Constants.DRIVETRAIN_TRANSFORM_KD),
      new PIDController(Constants.DRIVETRAIN_TRANSFORM_KP, Constants.DRIVETRAIN_TRANSFORM_KI, Constants.DRIVETRAIN_TRANSFORM_KD), 
                                                //Rotation PID controller, using 0 for all uses feedforwards, TO BE TUNED in Constants
      new PIDController(Constants.DRIVETRAIN_ROTATE_KP, Constants.DRIVETRAIN_ROTATE_KI, Constants.DRIVETRAIN_ROTATE_KD),
      Constants.MAX_AUTONOMOUS_WHEEL_SPEED,     //Max Wheel Speed
      output -> { drivetrain.setWheelSpeeds(output); }, 
      drivetrain                                //Requirements
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
