// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Arm.A_extendTo;
import frc.robot.commands.Arm.A_pivotToSLOW;
import frc.robot.commands.Grabber.G_PneumaticsOpen;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;

public class EngageArm extends SequentialCommandGroup {
  

  Arm s_arm;
  Grabber s_grabber;
  A_extendTo extendTo;

  //One-Shot Commmand
  //Opens Pnumatics (If they aren't already)
  //Swings arm out to X Degrees (Such that it's just beyond the frame perimeter)
  //Extends arm to the floor. (This is the default position.)
  public EngageArm(Arm arm, Grabber grabber) {
    addRequirements(arm, grabber);

    s_arm = arm;
    s_grabber = grabber;
    
    addCommands(
      new G_PneumaticsOpen(s_grabber),
      new ExtendArmToDefault(arm)
    );
  }
}
