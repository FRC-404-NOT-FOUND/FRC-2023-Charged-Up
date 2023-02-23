// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.A_extendTo;
import frc.robot.commands.Arm.A_pivotToSLOW;
import frc.robot.subsystems.Arm;

public class ExtendArmToDefault extends SequentialCommandGroup {
  public ExtendArmToDefault(Arm arm) {
    addCommands(new A_pivotToSLOW(0, arm), new A_extendTo(0, arm));  // Replace with angle and length of initial position
  }
}
