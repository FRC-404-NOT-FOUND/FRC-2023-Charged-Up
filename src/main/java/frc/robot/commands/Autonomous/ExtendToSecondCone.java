// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.A_extendTo;
import frc.robot.commands.Arm.A_pivotTo;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;

public class ExtendToSecondCone extends SequentialCommandGroup {
  public ExtendToSecondCone(Arm arm) {
    addCommands(
    // Replace with angle and length of cube
      new A_pivotTo(Constants.SECOND_CONE_ANGLE, arm), 
      new A_extendTo(Constants.SECOND_CONE_EXTENSION, arm)
    ); 
    addRequirements(arm.getExtension(), arm.getPivot());
  }
}