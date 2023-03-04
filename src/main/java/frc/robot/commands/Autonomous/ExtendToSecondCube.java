// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.A_extendTo;
import frc.robot.commands.Arm.A_pivotTo;
import frc.robot.subsystems.Arm;

public class ExtendToSecondCube extends SequentialCommandGroup {
  public ExtendToSecondCube(Arm arm) {
    addCommands(
    // Replace with angle and length of cube
      new A_pivotTo(Constants.SECOND_CUBE_ANGLE, arm), 
      new A_extendTo(Constants.SECOND_CUBE_EXTENSION, arm)
    ); 
    addRequirements(arm.getExtension(), arm.getPivot());
  }
}
