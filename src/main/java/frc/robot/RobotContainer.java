// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Arm.A_extendContinuous;
import frc.robot.commands.Arm.A_pivotContinuous;
import frc.robot.commands.Grabber.G_Spit;
import frc.robot.commands.Grabber.G_Succ;
import frc.robot.commands.MecanumDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Grabber;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain s_drivetrain = new Drivetrain();
  private final Arm s_arm = new Arm();
  private final Grabber s_grabber = new Grabber();

  //OI and Buttons
  private final Trigger oi_aExtend = new Trigger(() -> OI.gamepad.getRightBumper());
  private final Trigger oi_aRetract = new Trigger(() -> OI.gamepad.getLeftBumper());
  private final Trigger oi_aRaise = new Trigger(() -> OI.gamepad.getAButton());
  private final Trigger oi_aLower = new Trigger(() -> OI.gamepad.getBButton());

  //Dpad down InputExample. (Up == 0, Goes CW around by units of degrees.)
  //private final Trigger oi_AgoToDefault = new Trigger(() -> OI.gamepad.getPOV() == 180 ? true : false);

  private final Trigger oi_gCompressorToggle = new Trigger(() -> OI.gamepad.getStartButton());
  private final Trigger oi_gPneumaticsToggle = new Trigger(() -> OI.gamepad.getXButton());
  private final Trigger oi_gSucc = new Trigger(() -> OI.gamepad.getRightTriggerAxis() > 0.1 ? true : false);
  private final Trigger oi_gSpit = new Trigger(() -> OI.gamepad.getLeftTriggerAxis() > 0.1 ? true : false);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_drivetrain.setDefaultCommand(new MecanumDrive(
      s_drivetrain, 
      () -> OI.gamepad.getRawAxis(Constants.GAMEPAD_LEFT_STICK_X), 
      () -> OI.gamepad.getRawAxis(Constants.GAMEPAD_LEFT_STICK_Y),
      () -> OI.gamepad.getRawAxis(Constants.GAMEPAD_RIGHT_STICK_X)
    ));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //oi_aExtend.onTrue(new A_extendTo(Constants.EXTENSION_WHEEL_MAX_POSITION, s_Arm));
    //oi_aRetract.onTrue(new A_extendTo(Constants.EXTENSION_WHEEL_MIN_POSITION, s_Arm));

    oi_aExtend.whileTrue(new A_extendContinuous(s_arm, 1));
    oi_aRetract.whileTrue(new A_extendContinuous(s_arm, -1));

    oi_aRaise.whileTrue(new A_pivotContinuous(s_arm, 1));
    oi_aLower.whileTrue(new A_pivotContinuous(s_arm, -1));

    oi_gCompressorToggle.toggleOnTrue(Commands.startEnd(
      s_grabber::turnCompressorOn,
      s_grabber::turnCompressorOff,
      s_grabber
    ));

    oi_gPneumaticsToggle.toggleOnTrue(Commands.startEnd(
      s_grabber::pneumaticsClose,
      s_grabber::pneumaticsOpen,
      s_grabber
    ));

    oi_gSucc.whileTrue(new G_Succ(s_grabber));
    oi_gSpit.whileTrue(new G_Spit(s_grabber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   *
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
*/
}
