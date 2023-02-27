// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.A_extendTo;
import frc.robot.commands.Arm.A_pivotToSLOW;
import frc.robot.subsystems.Arm;
import frc.robot.Constants;

//Assume that we are coming from the Default Position.
public class ExtendToFirstPosition extends SequentialCommandGroup {
  public ExtendToFirstPosition(Arm arm) {
    addCommands(
      // Replace with angle and length of cube
      new A_pivotToSLOW(Constants.FIRST_POSITION_ANGLE, arm),
      new A_extendTo(Constants.FIRST_POSITION_EXTENSION, arm)
    );  
  }
}
