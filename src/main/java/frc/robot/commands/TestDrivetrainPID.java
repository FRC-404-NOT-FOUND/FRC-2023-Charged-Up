// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class TestDrivetrainPID extends CommandBase {
  public Drivetrain drivetrain;
  public ShuffleboardTab tab = Shuffleboard.getTab("PID_DrivetrainTest");
  
  public PIDController pid = new PIDController(0, 0, 0);

  public PIDController pid_FL = new PIDController(Constants.FRONT_LEFT_MOTOR_KP, Constants.FRONT_LEFT_MOTOR_KI, Constants.FRONT_LEFT_MOTOR_KD);
  public PIDController pid_FR = new PIDController(Constants.FRONT_RIGHT_MOTOR_KP, Constants.FRONT_RIGHT_MOTOR_KI, Constants.FRONT_RIGHT_MOTOR_KD);
  public PIDController pid_BL = new PIDController(Constants.BACK_LEFT_MOTOR_KP, Constants.BACK_LEFT_MOTOR_KI, Constants.BACK_LEFT_MOTOR_KD);
  public PIDController pid_BR = new PIDController(Constants.BACK_RIGHT_MOTOR_KP, Constants.BACK_RIGHT_MOTOR_KI, Constants.BACK_RIGHT_MOTOR_KD);

  public GenericEntry Velocity = tab.add("SetpointVelocity", 0)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .getEntry();
  public GenericEntry Motor = tab.add("Selected Motor", 0)
    .withWidget(BuiltInWidgets.kTextView)
    .getEntry();

  public GenericEntry kP = tab.add("kP", 0)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .getEntry();
  public GenericEntry kI = tab.add("kI", 0)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .getEntry();
  public GenericEntry kD = tab.add("kD", 0)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .getEntry();

  /** Creates a new TestDrivetrainPID. */
  public TestDrivetrainPID(Drivetrain d) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(d);
    drivetrain = d;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Test PID Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pid.setPID(kP.getDouble(0.0), kI.getDouble(0.0), kD.getDouble(0.0));
    switch((int) Motor.getInteger(0)){
      case(0):
        drivetrain
          .setWheelVoltages(pid.calculate(drivetrain.getKinematicWheelSpeeds().frontLeftMetersPerSecond, Velocity.getDouble(0.0)*2), 0.0, 0.0, 0.0);
        break;
      case(1):
        drivetrain
            .setWheelVoltages(0.0, pid.calculate(drivetrain.getKinematicWheelSpeeds().frontRightMetersPerSecond, Velocity.getDouble(0.0)*2), 0.0, 0.0);
        break;
      case(2):
        drivetrain
            .setWheelVoltages(0.0, 0.0, pid.calculate(drivetrain.getKinematicWheelSpeeds().rearLeftMetersPerSecond, Velocity.getDouble(0.0)*2), 0.0);
        break;
      case(3):
       drivetrain
            .setWheelVoltages(0.0, 0.0, 0.0, pid.calculate(drivetrain.getKinematicWheelSpeeds().rearRightMetersPerSecond, Velocity.getDouble(0.0)*2));
        break;
      case(4):
        drivetrain
          .setWheelVoltages(
            pid.calculate(drivetrain.getKinematicWheelSpeeds().frontLeftMetersPerSecond, Velocity.getDouble(0.0)*2.0), 
            pid.calculate(drivetrain.getKinematicWheelSpeeds().frontRightMetersPerSecond, Velocity.getDouble(0.0)*2.0), 
            pid.calculate(drivetrain.getKinematicWheelSpeeds().rearLeftMetersPerSecond, Velocity.getDouble(0.0)*2.0), 
            pid.calculate(drivetrain.getKinematicWheelSpeeds().rearRightMetersPerSecond, Velocity.getDouble(0.0)*2.0)
        );
        break;
      case(5):
        System.out.println(Velocity.getDouble(0));
        drivetrain
          .setWheelVoltages(
            pid_FL.calculate(drivetrain.getKinematicWheelSpeeds().frontLeftMetersPerSecond, Velocity.getDouble(0.0)*2.0), 
            pid_FR.calculate(drivetrain.getKinematicWheelSpeeds().frontRightMetersPerSecond, Velocity.getDouble(0.0)*2.0), 
            pid_BL.calculate(drivetrain.getKinematicWheelSpeeds().rearLeftMetersPerSecond, Velocity.getDouble(0.0)*2.0), 
            pid_BR.calculate(drivetrain.getKinematicWheelSpeeds().rearRightMetersPerSecond, Velocity.getDouble(0.0)*2.0)
        );
        break;

      default:
        break; 
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Wuh-oh.");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
