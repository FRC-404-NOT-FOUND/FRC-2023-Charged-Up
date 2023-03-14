// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;

public class TestPathPID extends CommandBase {
  Drivetrain drivetrain;
  public ShuffleboardTab tab = Shuffleboard.getTab("PID_PathTest");
  public GenericEntry restartPath = tab.add("Restart Path", false)
    .withWidget(BuiltInWidgets.kToggleButton)
    .getEntry();

  public GenericEntry cancelPath = tab.add("Cancel Path", false)
  .withWidget(BuiltInWidgets.kToggleButton)
  .getEntry();

  public GenericEntry path = tab.add("Path", "Horizontal")
    .withWidget(BuiltInWidgets.kComboBoxChooser)
    .getEntry();

  public boolean isFollowing = false;
  
  public PIDController horizontalPID = new PIDController(0, 0, 0);
  public PIDController verticalPID = new PIDController(0, 0, 0);
  public PIDController rotatePID = new PIDController(0, 0, 0);

  public Command sequence;

  public GenericEntry velocity = tab.add("Maximum Velocity", 2)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .getEntry();

  public GenericEntry accel = tab.add("Maximum Acceleration", 1)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .getEntry();

  /** Creates a new TestDrivetrainPID. */
  public TestPathPID(Drivetrain d) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(d);
    drivetrain = d;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Test PID Init");
  }

  final PathPlannerTrajectory traj = PathPlanner.loadPath(path.getString("Horizontal"), new PathConstraints(velocity.getDouble(0), accel.getDouble(0)));

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!isFollowing) {
      var followCommand = new PPMecanumControllerCommand(
        traj,                                     //PathPlannerTrajectory
        () -> drivetrain.getCurrentPose(),        //Pose Supplier (GETS THE CURRENT POSE EVERY TIME)
        drivetrain.getKinematics(),               //Kinematics of robot
        //X and Y PID COntrollers, using 0 for all uses feedforwards, TO BE TUNED in Constants
        verticalPID,
        horizontalPID, 
        //Rotation PID controller, using 0 for all uses feedforwards, TO BE TUNED in Constants
        rotatePID,
        4.5,     //Max Wheel Speed
        output -> { drivetrain.setKinematicWheelSpeeds(output); },
        drivetrain                           //Requirements
      );

      sequence = Commands.sequence(
        Commands.runOnce(() -> isFollowing = true),
        followCommand,
        Commands.runOnce(() -> isFollowing = false)
      );
    }

    if (restartPath.getBoolean(false)) {
      drivetrain.resetOdometry(traj.getInitialHolonomicPose());
      sequence.schedule();
      restartPath.setBoolean(false);
    }

    if (cancelPath.getBoolean(false)) {
      sequence.cancel();
      isFollowing = false;
      cancelPath.setBoolean(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Wuh-oh.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
