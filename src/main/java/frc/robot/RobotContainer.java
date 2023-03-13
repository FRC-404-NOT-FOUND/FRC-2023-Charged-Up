// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.MecanumDrive;
import frc.robot.commands.autonomous.AutonomousCommandSimple;
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
    //private final A_Extension sA_Extension = new A_Extension();
    //private final A_Pivot sA_Pivot = new A_Pivot();
  private final Grabber s_grabber = new Grabber();
    //private final G_Hopper sG_Hopper = new G_Hopper();
    //private final G_Intake sG_Intake = new G_Intake();

  private final MecanumDrive mecanumDrive = new MecanumDrive(
    s_drivetrain, 
    () -> OI.gamepad.getRawAxis(Constants.GAMEPAD_LEFT_STICK_X), 
    () -> OI.gamepad.getRawAxis(Constants.GAMEPAD_LEFT_STICK_Y),
    () -> OI.gamepad.getRawAxis(Constants.GAMEPAD_RIGHT_STICK_X),
    true
  );

  private final MecanumDrive mecanumDriveFine = new MecanumDrive(
    s_drivetrain, 
    () -> OI.gamepad.getRawAxis(Constants.GAMEPAD_LEFT_STICK_X) * Constants.DRIVETRAIN_FINE_CONTROL, 
    () -> OI.gamepad.getRawAxis(Constants.GAMEPAD_LEFT_STICK_Y) * Constants.DRIVETRAIN_FINE_CONTROL,
    () -> OI.gamepad.getRawAxis(Constants.GAMEPAD_RIGHT_STICK_X) * Constants.DRIVETRAIN_FINE_CONTROL,
    false
  );

  private final Trigger oi_driveFine = new Trigger(() -> OI.gamepad.getLeftStickButton());

  //OI and Buttons
  private final Trigger oi_aExtend = new Trigger(() -> OI.gamepad.getRightBumper());
  private final Trigger oi_aRetract = new Trigger(() -> OI.gamepad.getLeftBumper());
  private final Trigger oi_aRaise = new Trigger(() -> OI.gamepad.getRightTriggerAxis() > 0.1 ? true : false);
  private final Trigger oi_aLower = new Trigger(() -> OI.gamepad.getLeftTriggerAxis() > 0.1 ? true : false);

  //Dpad down InputExample. (Up == 0, Goes CW around by units of degrees.)
  private final Trigger oi_defaultPosition = new Trigger(() -> OI.gamepad.getYButton());
  private final Trigger oi_cube1Place = new Trigger(() -> OI.gamepad.getPOV() == 180 ? true : false);
  private final Trigger oi_cube2Place = new Trigger(() -> OI.gamepad.getPOV() == 90 ? true : false);
  private final Trigger oi_cone1Place = new Trigger(() -> OI.gamepad.getPOV() == 270 ? true : false);
  private final Trigger oi_cone2Place = new Trigger(() -> OI.gamepad.getPOV() == 0 ? true : false);
  

  private final Trigger oi_gCompressorToggle = new Trigger(() -> OI.gamepad.getStartButton());
  private final Trigger oi_gPneumaticsToggle = new Trigger(() -> OI.gamepad.getXButton());
  private final Trigger oi_gSucc = new Trigger(() -> OI.gamepad.getAButton());
  private final Trigger oi_gSpit = new Trigger(() -> OI.gamepad.getBButton());

 // private final Trigger oi_iArduinoReconnect = new Trigger(() -> OI.gamepad.getBackButtonPressed());
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    s_drivetrain.setDefaultCommand(mecanumDrive);

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
    // oi_cubePlace.onTrue(new MoveToAprilTag(s_drivetrain));
    // oi_cubePlace.onFalse(new MecanumDrive(
    //   s_drivetrain, 
    //   () -> OI.gamepad.getRawAxis(Constants.GAMEPAD_LEFT_STICK_X), 
    //   () -> OI.gamepad.getRawAxis(Constants.GAMEPAD_LEFT_STICK_Y),
    //   () -> OI.gamepad.getRawAxis(Constants.GAMEPAD_RIGHT_STICK_X)
    // ));

    // oi_iArduinoReconnect.whileTrue(new TryReconnectArduino(
    //   imu
    // ));

    oi_aExtend.whileTrue(s_arm.extendContinuousCommand());
    oi_aRetract.whileTrue(s_arm.retractContinuousCommand());

    oi_aRaise.whileTrue(s_arm.raiseContinuousCommand(() -> OI.gamepad.getRightTriggerAxis()));
    oi_aLower.whileTrue(s_arm.lowerContinuousCommand(() -> OI.gamepad.getLeftTriggerAxis()));

    s_grabber.turnCompressorOn();
    // oi_gCompressorToggle.toggleOnFalse(s_grabber.toggleCompressorCommand());
    oi_gPneumaticsToggle.toggleOnTrue(s_grabber.toggleGrabberCommand());

    oi_gSucc.whileTrue(s_grabber.intakeCommand());
    oi_gSpit.whileTrue(s_grabber.spitCommand());

    //Toggles Fine Control
    oi_driveFine.toggleOnTrue(mecanumDriveFine);

    oi_defaultPosition.onTrue(s_arm.moveToDefault());
    oi_cone1Place.onTrue(s_arm.moveArmTo(Constants.FIRST_CONE_ANGLE, Constants.FIRST_CONE_EXTENSION).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    oi_cone2Place.onTrue(s_arm.moveArmTo(Constants.SECOND_CONE_ANGLE, Constants.SECOND_CONE_EXTENSION).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    oi_cube1Place.onTrue(s_arm.moveArmTo(Constants.FIRST_CUBE_ANGLE, Constants.FIRST_CUBE_EXTENSION).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    oi_cube2Place.onTrue(s_arm.moveArmTo(Constants.SECOND_CUBE_ANGLE, Constants.SECOND_CUBE_EXTENSION).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   *
   * */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new AutonomousCommandSimple(s_drivetrain, s_arm, s_grabber);
  }

  public Drivetrain getDrivetrain(){
    return s_drivetrain;
  }

  public Grabber getGrabber(){
    return s_grabber;
  }
}
