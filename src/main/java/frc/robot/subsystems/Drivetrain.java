// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;

import com.ctre.phoenixpro.hardware.CANcoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  //Motors
  private VictorSP frontLeftMotor = new VictorSP(Constants.FRONT_LEFT_MOTOR_INDEX);
  private VictorSP frontRightMotor = new VictorSP(Constants.FRONT_RIGHT_MOTOR_INDEX);
  private VictorSP backLeftMotor = new VictorSP(Constants.BACK_LEFT_MOTOR_INDEX);
  private VictorSP backRightMotor = new VictorSP(Constants.BACK_RIGHT_MOTOR_INDEX);
  //Encoders
  private CANcoder frontLeftEncoder = new CANcoder(Constants.FRONT_LEFT_ENCODER_INDEX);
  private CANcoder frontRightEncoder = new CANcoder(Constants.FRONT_RIGHT_ENCODER_INDEX);
  private CANcoder backLeftEncoder = new CANcoder(Constants.BACK_LEFT_ENCODER_INDEX);
  private CANcoder backRightEncoder = new CANcoder(Constants.BACK_RIGHT_ENCODER_INDEX);

  //https://github.wpilib.org/allwpilib/docs/beta/java/edu/wpi/first/math/kinematics/MecanumDriveKinematics.html
  //Input relative distance for each wheel from middle of robot, IN METERS!
  private MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
    Constants.FRONT_LEFT_WHEEL_TO_CENTER, 
    Constants.FRONT_RIGHT_WHEEL_TO_CENTER, 
    Constants.BACK_LEFT_WHEEL_TO_CENTER, 
    Constants.BACK_RIGHT_WHEEL_TO_CENTER
  );

  private MecanumDriveOdometry odometry;

  private MecanumDrive mDrive;

  public Drivetrain() {
    mDrive = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

    odometry = new MecanumDriveOdometry(kinematics, getGyroAngle(), getWheelPositions() /*, INITIAL POSE!!! */);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void setDefaultCommand(Command defaultCommand) {
    super.setDefaultCommand(defaultCommand);
  }

  public void driveCartesian(double xSpeed, double ySpeed, double zRotation) {
    mDrive.driveCartesian(xSpeed, ySpeed, zRotation);
  }
  
  public MecanumDriveKinematics getKinematics(){
    return kinematics;
  }
  
  //This should be the main pose. Here, we should combine
  //The IDEAL data from Odometry, 
  //The MEASURED data from Vision, 
  //And maybe the gyro?
  //We should also use some kind of filter/average within this to get an accurate result.
  public Pose2d getCurrentPose(){
    return null;
  }

  ////////////////////////////////////////////////
  /////        ENCODERS and SENSORS          /////
  ////////////////////////////////////////////////
  //NOTE: These do not work yet. Conversion constants must be made to make these values function as intended
  //Also a refresh loop must be implemented in order to keep updating the value

  //These are in rotations (i.e. "Distance Driven"), will have to convert to meters (Using a constant value)
  public MecanumDriveWheelPositions getWheelPositions(){
    return new MecanumDriveWheelPositions(
      frontLeftEncoder.getPosition().getValue() * Constants.ROTATIONS_TO_METERS, 
      frontRightEncoder.getPosition().getValue() * Constants.ROTATIONS_TO_METERS, 
      backLeftEncoder.getPosition().getValue() * Constants.ROTATIONS_TO_METERS, 
      backRightEncoder.getPosition().getValue() * Constants.ROTATIONS_TO_METERS
    );
  }
  //These are in rotations a second, will have to convert to meters per second. (Using a constant value)
  public MecanumDriveWheelSpeeds getWheelSpeeds(){
    return new MecanumDriveWheelSpeeds(
      frontLeftEncoder.getVelocity().getValue() * Constants.ROTATIONS_TO_METERS, 
      frontRightEncoder.getVelocity().getValue() * Constants.ROTATIONS_TO_METERS, 
      backLeftEncoder.getVelocity().getValue() * Constants.ROTATIONS_TO_METERS, 
      backRightEncoder.getVelocity().getValue() * Constants.ROTATIONS_TO_METERS
    );
  }
  
  //TO BE IMPLEMENTED
  public Rotation2d getGyroAngle(){
    //Here we'll send a signal through the GYRO (maybe different subsytem) to the Arduino, asking it for current information.
    //It then sends that requested data back to us
    return null;
  }

  ////////////////////////////////////////////////
  ///// ODOMETRY (Purely Encoder/Gyro Based) /////
  ////////////////////////////////////////////////

  //Gets the current position in meters.
  public Pose2d getOdometryPose(){
    return odometry.getPoseMeters();
  }

  //Updates the current odometry, and returns the newest pose.
  public Pose2d updateOdometry(Rotation2d angle, MecanumDriveWheelPositions wheelPositions){
    return odometry.update(angle, wheelPositions);
  }

  //Resets the odometry to the selected values.
  public void resetOdometry(Rotation2d angle, MecanumDriveWheelPositions wheelPositions, Pose2d pose){
    odometry.resetPosition(angle, wheelPositions, pose);
  }


  ////////////////////////////////////////////////
  /////               Setters                /////
  ////////////////////////////////////////////////

//Sets individual wheel speeds to the mecanum drive.
  public void setWheelSpeeds(MecanumDriveWheelSpeeds wheelSpeeds){
    ChassisSpeeds cartesian = kinematics.toChassisSpeeds(wheelSpeeds);

    MecanumDrive.driveCartesianIK(cartesian.vxMetersPerSecond, cartesian.vyMetersPerSecond, cartesian.omegaRadiansPerSecond);
  }
  
  public void setInvertfrontLeftMotor(boolean isInverted) {
    this.frontLeftMotor.setInverted(isInverted);
  }
  public void setInvertfrontRightMotor(boolean isInverted) {
    this.frontRightMotor.setInverted(isInverted);
  }
  public void setInvertBackLeftMotor(boolean isInverted) {
    this.backLeftMotor.setInverted(isInverted);
  }
  public void setInvertBackRightMotor(boolean isInverted) {
    this.backRightMotor.setInverted(isInverted);
  }

}
