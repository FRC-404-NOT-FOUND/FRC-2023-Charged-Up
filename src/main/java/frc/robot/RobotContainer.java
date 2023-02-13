// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.A_extendTo;
import frc.robot.commands.MecanumDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.subArm.A_Extension;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain s_drivetrain = new Drivetrain();
  private final A_Extension s_aExtension = new A_Extension();

  //OI and Buttons
  private final Trigger oi_aExtendToMax = new JoystickButton(OI.gamepad, XboxController.Button.kRightBumper.value);
  private final Trigger oi_aExtendToMin = new JoystickButton(OI.gamepad, XboxController.Button.kLeftBumper.value);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_drivetrain.setInvertfrontRightMotor(true);
    s_drivetrain.setInvertBackLeftMotor(true);
    s_drivetrain.setDefaultCommand(
      new MecanumDrive(s_drivetrain, 
        () -> OI.gamepad.getRawAxis(Constants.GAMEPAD_LEFT_STICK_X), 
        () -> OI.gamepad.getRawAxis(Constants.GAMEPAD_LEFT_STICK_Y),
        () -> OI.gamepad.getRawAxis(Constants.GAMEPAD_RIGHT_STICK_X))
      );

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
    oi_aExtendToMax.whenActive(new A_extendTo(Constants.EXTENSION_WHEEL_MAX_POSITION, s_aExtension));
    oi_aExtendToMin.whenActive(new A_extendTo(Constants.EXTENSION_WHEEL_MIN_POSITION, s_aExtension));
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
