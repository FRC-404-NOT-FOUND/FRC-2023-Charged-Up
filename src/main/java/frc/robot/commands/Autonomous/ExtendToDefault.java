// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.A_extendTo;
import frc.robot.commands.Arm.A_pivotTo;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExtendToDefault extends SequentialCommandGroup {
  Arm s_arm;
  Grabber s_grabber;

  //One-Shot Command
  //Retracts Arm to 0.
  //Opens Pnumatics (If they aren't already)
  //Swings arm back to 0 Degrees (SLOWLY)
  public ExtendToDefault(Arm arm, Grabber grabber) {
    addRequirements(arm, grabber);

    s_arm = arm;
    s_grabber = grabber;
    
    addCommands(
      Commands.run(s_grabber::pneumaticsOpen, s_grabber),
      new A_extendTo(Constants.DEFAULT_EXTENSION, s_arm),
      new A_pivotTo(Constants.DEFAULT_ANGLE, s_arm)
    );
  }
}
