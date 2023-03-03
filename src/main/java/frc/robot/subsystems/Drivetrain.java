// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Time;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenixpro.hardware.CANcoder;

import edu.wpi.first.math.*;
import edu.wpi.first.math.estimator.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Limelight;

public class Drivetrain extends SubsystemBase {
  //Motors
  private VictorSP frontLeftMotor = new VictorSP(Constants.FRONT_LEFT_MOTOR_INDEX);
  private VictorSP frontRightMotor = new VictorSP(Constants.FRONT_RIGHT_MOTOR_INDEX);
  private VictorSP backLeftMotor = new VictorSP(Constants.BACK_LEFT_MOTOR_INDEX);
  private VictorSP backRightMotor = new VictorSP(Constants.BACK_RIGHT_MOTOR_INDEX);

  //Encoders
  private CANCoder frontLeftEncoder = new CANCoder(Constants.FRONT_LEFT_ENCODER_INDEX);
  private CANCoder frontRightEncoder = new CANCoder(Constants.FRONT_RIGHT_ENCODER_INDEX);
  private CANCoder backLeftEncoder = new CANCoder(Constants.BACK_LEFT_ENCODER_INDEX);
  private CANCoder backRightEncoder = new CANCoder(Constants.BACK_RIGHT_ENCODER_INDEX);

  //https://github.wpilib.org/allwpilib/docs/beta/java/edu/wpi/first/math/kinematics/MecanumDriveKinematics.html
  //Input relative distance for each wheel from middle of robot, IN METERS!
  private MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
    Constants.FRONT_LEFT_WHEEL_TO_CENTER, 
    Constants.FRONT_RIGHT_WHEEL_TO_CENTER, 
    Constants.BACK_LEFT_WHEEL_TO_CENTER, 
    Constants.BACK_RIGHT_WHEEL_TO_CENTER
  );

  private MecanumDriveOdometry odometry;
  Rotation2d currentRot = new Rotation2d(0);
  //Current pose = Odometry

  /*
  // This plant holds a state-space model of our drivetrain. This system has the following properties:
  //
  // States: [x, y, theta], in meters/degrees.
  // Inputs (what we can "put in"): [voltage], in volts.
  // Outputs (what we can measure): [x, y, theta] in meters.
  // private final LinearSystem<N2, N2, N2> m_mecanumLinearSystemPlant = LinearSystemId.createDrivetrainVelocitySystem(
  //   DCMotor.getCIM(4), 
  //   Constants.ROBOT_MASS_KG, 
  //   Constants.DRIVETRAIN_WHEEL_RADIUS, 
  //   Constants.FRONT_LEFT_WHEEL_TO_CENTER.getY(), 
  //   0, 
  //   0
  // );

  // //<State (of robot), Inputs (to plant), Outputs (From sensors)>
  // //For more information check: 
  // //    https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-observers.html
  // private final KalmanFilter<N2, N2, N2> m_kalmanFilter =
  //     new KalmanFilter<N2, N2, N2>(
  //       Nat.N2(), //State: [X, Y, Heading]
  //       Nat.N2(), //Output (From Limelight): [X, Y, Heading]
  //       m_mecanumLinearSystemPlant,
  //       null, 
  //       null, 
  //       0.02
  //   );
  */

  private MecanumDrive mDrive;

  private IMU imu;

  private MecanumDrivePoseEstimator poseEstimator;

  public Drivetrain() {
    frontLeftMotor.setInverted(false);
    frontRightMotor.setInverted(true);
    backLeftMotor.setInverted(false);
    backRightMotor.setInverted(true);

    mDrive = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

    imu = IMU.create();

    Pose2d initialPose = new Pose2d(0, 0, getGyroAngle());

    //Get starting pose from driverstation, where it is selected.
    if (Limelight.isValidTarget()) {
      initialPose = getVisionPose();
    } else {
      initialPose = new Pose2d(); // Possibly replace with possible values for start positions based on the map
    }

    //odometry = new MecanumDriveOdometry(kinematics, getGyroAngle(), getWheelPositions(), initialPose);

    poseEstimator = new MecanumDrivePoseEstimator(kinematics, getGyroAngle(), getWheelPositions(), initialPose);
  }

  @Override
  // Here, we should combine
  // The IDEAL data from Odometry, 
  // The MEASURED data from Vision and gyro, 
  // In the future, we should also use some kind of filter/average within this to get an accurate result.
  public void periodic() {
    //Update rotation first.
    //THEN update pose through odometry

    poseEstimator.update(getGyroAngle(), getWheelPositions());

    //System.out.println("Front left encoder position: " + frontLeftEncoder.getPosition().getValue());
    //System.out.println("Front right encoder position: " + frontRightEncoder.getPosition().getValue());
    //System.out.println("Back left encoder: " + backLeftEncoder.getPosition().getValue());
    //System.out.println("Back right encoder: " + backRightEncoder.getPosition().getValue());
    
    if (Limelight.isValidTarget() || Limelight.getPrimaryAprilTag() <= 0) {
      poseEstimator.addVisionMeasurement(getVisionPose(), Constants.timer.get());
    } 
  }

  @Override
  public void setDefaultCommand(Command defaultCommand) {
    super.setDefaultCommand(defaultCommand);
  }

  // The IDEAL version of mecanum Drive. Does not inplement wheelSpeed Offsets.
  // Automatically sets wheels within this method, doesn't return anything.
  // Positive Directions:
  // XSpeed = forward, YSpeed = Left, zRotation = CCW
  public void driveCartesian(double xSpeed, double ySpeed, double zRotation) {
    mDrive.driveCartesian(xSpeed, ySpeed, zRotation);
  }

  //Returns the wheelSpeeds (From -1 to 1.) from the parameters.
  public MecanumDrive.WheelSpeeds driveCartesianIK(double xSpeed, double ySpeed, double zRotation) {
    MecanumDrive.WheelSpeeds wheelSpeeds = MecanumDrive.driveCartesianIK(xSpeed, ySpeed, zRotation);

    return wheelSpeeds;
  }
  
  public MecanumDriveKinematics getKinematics(){
    return kinematics;
  }
  
  // This should be the main pose. 
  public Pose2d getCurrentPose(){
    return poseEstimator.getEstimatedPosition();
  }

  ////////////////////////////////////////////////
  /////        ENCODERS and SENSORS          /////
  ////////////////////////////////////////////////
  //NOTE: These do not work yet. Conversion constants must be made to make these values function as intended
  //Also a refresh loop must be implemented in order to keep updating the value

  //These are in rotations (i.e. "Distance Driven"), will have to convert to meters (Using a constant value)
  public MecanumDriveWheelPositions getWheelPositions(){
    return new MecanumDriveWheelPositions(
      frontLeftEncoder.getPosition() * Constants.ROTATIONS_TO_METERS, 
      frontLeftEncoder.getPosition() * Constants.ROTATIONS_TO_METERS, 
      frontLeftEncoder.getPosition() * Constants.ROTATIONS_TO_METERS, 
      frontLeftEncoder.getPosition() * Constants.ROTATIONS_TO_METERS
    );
  }

  //These are in rotations a second, will have to convert to meters per second. (Using a constant value)
  public MecanumDriveWheelSpeeds getWheelSpeeds(){
    return new MecanumDriveWheelSpeeds(
      frontLeftEncoder.getVelocity() * Constants.ROTATIONS_TO_METERS, 
      frontRightEncoder.getVelocity() * Constants.ROTATIONS_TO_METERS, 
      backLeftEncoder.getVelocity() * Constants.ROTATIONS_TO_METERS, 
      backRightEncoder.getVelocity() * Constants.ROTATIONS_TO_METERS
    );
  }
  
  // Returns a Rotation2d based on the Gyro's rotation z after calibration
  public Rotation2d getGyroAngle(){
    if (imu.isGyroReady()) {
    //Here we'll send a signal through the GYRO (maybe different subsytem) to the Arduino, asking it for current information.
    //It then sends that requested data back to us
      return new Rotation2d(imu.getGyroYaw());
    }
    
    return new Rotation2d();
  }

  ////////////////////////////////////////////////
  ///// ODOMETRY (Purely Encoder/Gyro Based) /////
  ////////////////////////////////////////////////

  //Updates the current odometry, and returns the newest pose.
  // public Pose2d updateOdometry(Rotation2d angle, MecanumDriveWheelPositions wheelPositions){
  //   return odometry.update(angle, wheelPositions);
  // }

  // //Resets the odometry to the selected values.
  // public void resetOdometry(Rotation2d angle, MecanumDriveWheelPositions wheelPositions, Pose2d pose){
  //   odometry.resetPosition(angle, wheelPositions, pose);
  // }

  ////////////////////////////////////////////////
  /////               Vision                 /////
  ////////////////////////////////////////////////

  // This uses the Limelight botpose x, y, and yaw values to determine the Pose2d of the robot
  // This should only be called if there is a valid target, as it updates the odometry position
  public Pose2d getVisionPose(){
    double[] pose = Limelight.getBotPose();
    Translation2d t = new Translation2d(pose[0], pose[1]); // x and y values for determining coordinates
    Rotation2d r = Rotation2d.fromDegrees(pose[5]); // yaw for determining rotation
    Pose2d p = new Pose2d(t, r);
    return p;
  }

  // Gets the current Rotation2d of the robot based on the botpose yaw
  public Rotation2d getVisionRotation(){
    double[] pose = Limelight.getBotPose();
    return Rotation2d.fromDegrees(pose[5]); // Important! Limelight returns in degrees, Rotation2d needs radians
  }


  ////////////////////////////////////////////////
  /////               Setters                /////
  ////////////////////////////////////////////////

  //From (-1 -> 1), Sets the POWER of each motor.
  public void setWheelSpeeds(MecanumDrive.WheelSpeeds wheelSpeeds){
    frontLeftMotor.set(wheelSpeeds.frontLeft);
    frontRightMotor.set(wheelSpeeds.frontRight);
    backLeftMotor.set(wheelSpeeds.rearLeft);
    backRightMotor.set(wheelSpeeds.rearRight);
  }

//NEEDS TO BE FINALIZED.
//Should be done using velocity PID. Cannot be done without it.
//Sets individual wheel speeds to the mecanum drive.
  public void setKinematicWheelSpeeds(MecanumDriveWheelSpeeds kinematicWheelSpeeds){
    MecanumDrive.WheelSpeeds wheelSpeeds = new MecanumDrive.WheelSpeeds();
    // Linear Velocity = AngularVelocity * Radius
    wheelSpeeds.frontLeft = (kinematicWheelSpeeds.frontLeftMetersPerSecond / Constants.DRIVETRAIN_WHEEL_RADIUS);
    wheelSpeeds.frontRight = (kinematicWheelSpeeds.frontRightMetersPerSecond / Constants.DRIVETRAIN_WHEEL_RADIUS);
    wheelSpeeds.rearLeft = (kinematicWheelSpeeds.rearLeftMetersPerSecond / Constants.DRIVETRAIN_WHEEL_RADIUS);
    wheelSpeeds.rearRight = (kinematicWheelSpeeds.rearRightMetersPerSecond / Constants.DRIVETRAIN_WHEEL_RADIUS);

    //setWheelSpeeds(wheelSpeeds);
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
