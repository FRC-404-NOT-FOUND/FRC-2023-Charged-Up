// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Grabber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousCommandSimple extends SequentialCommandGroup {
  /** Creates a new AutonomousCommandSimple. */
  public AutonomousCommandSimple(Drivetrain drivetrain, Arm arm, Grabber grabber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new DriveAlongPath(PathPlanner.loadPath("Autonomous Plan", new PathConstraints(3, 2)), drivetrain, isFinished())
      Commands.run(() -> drivetrain.driveCartesian(0.3, 0, 0))
    );
  }
}
