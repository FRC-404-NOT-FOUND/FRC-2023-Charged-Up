// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
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
  private WPI_CANCoder frontLeftEncoder = new WPI_CANCoder(Constants.FRONT_LEFT_ENCODER_INDEX);
  private WPI_CANCoder frontRightEncoder = new WPI_CANCoder(Constants.FRONT_RIGHT_ENCODER_INDEX);
  private WPI_CANCoder backLeftEncoder = new WPI_CANCoder(Constants.BACK_LEFT_ENCODER_INDEX);
  private WPI_CANCoder backRightEncoder = new WPI_CANCoder(Constants.BACK_RIGHT_ENCODER_INDEX);

  //https://github.wpilib.org/allwpilib/docs/beta/java/edu/wpi/first/math/kinematics/MecanumDriveKinematics.html
  //Input relative distance for each wheel from middle of robot, IN METERS!
  private MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
    Constants.FRONT_LEFT_WHEEL_TO_CENTER, 
    Constants.FRONT_RIGHT_WHEEL_TO_CENTER, 
    Constants.BACK_LEFT_WHEEL_TO_CENTER, 
    Constants.BACK_RIGHT_WHEEL_TO_CENTER
  );

  private PIDController frontLeftMotorPID = new PIDController(Constants.FRONT_LEFT_MOTOR_KP, Constants.FRONT_LEFT_MOTOR_KI, Constants.FRONT_LEFT_MOTOR_KD);
  private PIDController frontRightMotorPID = new PIDController(Constants.FRONT_RIGHT_MOTOR_KP, Constants.FRONT_RIGHT_MOTOR_KI, Constants.FRONT_RIGHT_MOTOR_KD);
  private PIDController backLeftMotorPID = new PIDController(Constants.BACK_LEFT_MOTOR_KP, Constants.BACK_LEFT_MOTOR_KI, Constants.BACK_LEFT_MOTOR_KD);
  private PIDController backRightMotorPID = new PIDController(Constants.BACK_RIGHT_MOTOR_KP, Constants.BACK_RIGHT_MOTOR_KI, Constants.BACK_RIGHT_MOTOR_KD);
  //private PIDController rotatePID = new PIDController(Constants.DRIVETRAIN_ROTATE_KP, Constants.DRIVETRAIN_ROTATE_KI, Constants.DRIVETRAIN_ROTATE_KD);

  private SimpleMotorFeedforward frontLeftMotorFF = new SimpleMotorFeedforward(Constants.FRONT_LEFT_MOTOR_FF_KS, Constants.FRONT_LEFT_MOTOR_FF_KV, Constants.FRONT_LEFT_MOTOR_FF_KA);
  private SimpleMotorFeedforward frontRightMotorFF = new SimpleMotorFeedforward(Constants.FRONT_RIGHT_MOTOR_FF_KS, Constants.FRONT_RIGHT_MOTOR_FF_KV, Constants.FRONT_RIGHT_MOTOR_FF_KA);
  private SimpleMotorFeedforward backLeftMotorFF = new SimpleMotorFeedforward(Constants.BACK_LEFT_MOTOR_FF_KS, Constants.BACK_LEFT_MOTOR_FF_KV, Constants.BACK_LEFT_MOTOR_FF_KA);
  private SimpleMotorFeedforward backRightMotorFF = new SimpleMotorFeedforward(Constants.BACK_RIGHT_MOTOR_FF_KS, Constants.BACK_RIGHT_MOTOR_FF_KV, Constants.BACK_RIGHT_MOTOR_FF_KA);

  private MecanumDrive mDrive;

  private SlewRateLimiter xLimiter = new SlewRateLimiter(1.8);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(1.8);
  private SlewRateLimiter zLimiter = new SlewRateLimiter(1.9);

  private Field2d field2d = new Field2d();

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

    poseEstimator = new MecanumDrivePoseEstimator(
      kinematics,
      getGyroAngle(),
      getKinematicWheelPositions(),
      initialPose,
      VecBuilder.fill(0.2, 0.2, 0.2),
      VecBuilder.fill(0.1, 0.1, 0.1)
    );
  }

  @Override
  public void periodic() {
    var kinPos = getKinematicWheelPositions();
    var angle = getGyroAngle();

    poseEstimator.update(angle, kinPos);

    if (Limelight.isValidTarget() && Limelight.getPrimaryAprilTag() > 0) {
      poseEstimator.addVisionMeasurement(getVisionPose(), Timer.getFPGATimestamp());
    }

    field2d.setRobotPose(getCurrentPose());

    SmartDashboard.putNumber("Front Left Encoder position", frontLeftEncoder.getPosition());
    SmartDashboard.putNumber("Front Right Encoder position", frontRightEncoder.getPosition());
    SmartDashboard.putNumber("Back Left Encoder position", backLeftEncoder.getPosition());
    SmartDashboard.putNumber("Back Right Encoder position", backRightEncoder.getPosition());

    SmartDashboard.putNumber("Front Left Encoder Meters", kinPos.frontLeftMeters);
    SmartDashboard.putNumber("Front Right Encoder Meters", kinPos.frontRightMeters);
    SmartDashboard.putNumber("Back Left Encoder Meters", kinPos.rearLeftMeters);
    SmartDashboard.putNumber("Back Right Encoder Meters", kinPos.rearRightMeters);

    SmartDashboard.putNumber("Front Left Encoder Velocity", frontLeftEncoder.getVelocity());
    SmartDashboard.putNumber("Front Right Encoder Velocity", frontRightEncoder.getVelocity());
    SmartDashboard.putNumber("Back Left Encoder Velocity", backLeftEncoder.getVelocity());
    SmartDashboard.putNumber("Back Right Encoder Velocity", backRightEncoder.getVelocity());

    SmartDashboard.putNumber("Gyro Angle", angle.getDegrees());

    SmartDashboard.putData(field2d);
  }

  // DO NOT USE EXCEPT IN TEST!!!
  public void resetOdometry(Pose2d pose) {
    frontLeftEncoder.setPosition(0);
    frontRightEncoder.setPosition(0);
    backLeftEncoder.setPosition(0);
    backRightEncoder.setPosition(0);
    poseEstimator.resetPosition(new Rotation2d(), new MecanumDriveWheelPositions(), pose);
  }

  @Override
  public void setDefaultCommand(Command defaultCommand) {
    super.setDefaultCommand(defaultCommand);
  }

  // Drives the robot along the given path
  public Command followPath(String path) {
    return new PPMecanumControllerCommand(
      PathPlanner.loadPath(path, new PathConstraints(Constants.MAX_AUTONOMOUS_WHEEL_VELOCITY, Constants.MAX_AUTONOMOUS_WHEEL_ACCEL)),                               //PathPlannerTrajectory
      () -> getCurrentPose(),        //Pose Supplier (GETS THE CURRENT POSE EVERY TIME)
      getKinematics(),               //Kinematics of robot
      //X and Y PID COntrollers, using 0 for all uses feedforwards, TO BE TUNED in Constants
      new PIDController(Constants.DRIVETRAIN_TRANSFORM_KPx, Constants.DRIVETRAIN_TRANSFORM_KIx, Constants.DRIVETRAIN_TRANSFORM_KDx),
      new PIDController(Constants.DRIVETRAIN_TRANSFORM_KPy, Constants.DRIVETRAIN_TRANSFORM_KIy, Constants.DRIVETRAIN_TRANSFORM_KDy), 
      //Rotation PID controller, using 0 for all uses feedforwards, TO BE TUNED in Constants
      new PIDController(Constants.DRIVETRAIN_ROTATE_KP, Constants.DRIVETRAIN_ROTATE_KI, Constants.DRIVETRAIN_ROTATE_KD),
      4,     //Max Wheel Speed
      output -> { setKinematicWheelSpeeds(output); },
      this                            //Requirements
    );
  }

  // Attempts to balance the robot on the charge station
  public Command balance() {
    return new ProfiledPIDCommand(
      // The ProfiledPIDController used by the command
      new ProfiledPIDController(
          // The PID gains
          Constants.DRIVETRAIN_TRANSFORM_KPx,
          Constants.DRIVETRAIN_TRANSFORM_KIx,
          Constants.DRIVETRAIN_TRANSFORM_KDx,
          // The motion profile constraints
          new TrapezoidProfile.Constraints(Constants.MAX_AUTONOMOUS_WHEEL_VELOCITY, Constants.MAX_AUTONOMOUS_WHEEL_ACCEL)),
      // This should return the measurement
      () -> imu.getGyroPitch(),
      // This should return the goal (can also be a constant)
      // Tl;dr THIS IS THE TARGET, and the pitch should be zero (or close to it.).
      () -> new TrapezoidProfile.State(),
      // This uses the output
      (output, setpoint) -> {
        // Use the output (and setpoint, if desired) here
        setWheelSpeeds(new WheelSpeeds(output, output, output, output));
      }
    );
  }

  // The IDEAL version of mecanum Drive. Does not implement wheelSpeed Offsets.
  // Automatically sets wheels within this method, doesn't return anything.
  // Positive Directions:
  // XSpeed = forward, YSpeed = Left, zRotation = CCW
  public void driveCartesian(double xSpeed, double ySpeed, double zRotation) {
    driveCartesian(xSpeed, ySpeed, zRotation, false);
  }

  public void driveCartesian(double xSpeed, double ySpeed, double zRotation, boolean fieldRelative) {
    xSpeed = MathUtil.applyDeadband(xSpeed, 0.1);
    ySpeed = MathUtil.applyDeadband(ySpeed, 0.1);
    //double rate = MathUtil.applyDeadband(imu.getGyroRate(), 0.1);

    double x = xLimiter.calculate(xSpeed);
    double y = yLimiter.calculate(ySpeed);

    zRotation = MathUtil.applyDeadband(zRotation, 0.05);
    //zRotation = rotatePID.calculate(rate, zRotation);
    double z = zLimiter.calculate(zRotation);

    var speeds = kinematics.toWheelSpeeds(
      fieldRelative ?
        ChassisSpeeds.fromFieldRelativeSpeeds(x, y, z, getGyroAngle()) :
        new ChassisSpeeds(x, y, z)
    );

    speeds.frontLeftMetersPerSecond *= 4.5;
    speeds.frontRightMetersPerSecond *= 4.5;
    speeds.rearLeftMetersPerSecond *= 4.5;
    speeds.rearRightMetersPerSecond *= 4.5;
    speeds.desaturate(4.5);

    setKinematicWheelSpeeds(speeds);
    mDrive.feed();
    //mDrive.driveCartesian(xSpeed, ySpeed, zRotation, fieldRelative ? getGyroAngle() : new Rotation2d());
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

  //These are in Degrees
  public MecanumDriveWheelPositions getAngularWheelPositions(){
    return new MecanumDriveWheelPositions(
      frontLeftEncoder.getPosition(), 
      frontRightEncoder.getPosition(), 
      backLeftEncoder.getPosition(), 
      backRightEncoder.getPosition()    
    );
  }

  //These are in Meters
  public MecanumDriveWheelPositions getKinematicWheelPositions(){
    return new MecanumDriveWheelPositions(
      frontLeftEncoder.getPosition() * Constants.DEGREES_TO_METERS, 
      frontRightEncoder.getPosition() * Constants.DEGREES_TO_METERS, 
      backLeftEncoder.getPosition() * Constants.DEGREES_TO_METERS, 
      backRightEncoder.getPosition() * Constants.DEGREES_TO_METERS
    );
  }

  //These are in Degrees/Second
  public MecanumDriveWheelSpeeds getAngularWheelSpeeds(){
    return new MecanumDriveWheelSpeeds(
      frontLeftEncoder.getVelocity(), 
      frontRightEncoder.getVelocity(), 
      backLeftEncoder.getVelocity(), 
      backRightEncoder.getVelocity()
    );
  }

  //These are in Meters/Second.
  public MecanumDriveWheelSpeeds getKinematicWheelSpeeds(){
    return new MecanumDriveWheelSpeeds(
      frontLeftEncoder.getVelocity() * Constants.DEGREES_TO_METERS, 
      frontRightEncoder.getVelocity() * Constants.DEGREES_TO_METERS, 
      backLeftEncoder.getVelocity() * Constants.DEGREES_TO_METERS, 
      backRightEncoder.getVelocity() * Constants.DEGREES_TO_METERS
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
  /////               Vision                 /////
  ////////////////////////////////////////////////

  // This uses the Limelight botpose x, y, and yaw values to determine the Pose2d of the robot
  // This should only be called if there is a valid target, as it updates the odometry position
  public Pose2d getVisionPose(){
    double[] pose = Limelight.getBotPose();
    Translation2d t = new Translation2d(pose[0], pose[1]); // x and y values for determining coordinates
    Rotation2d r = Rotation2d.fromDegrees(pose[5]); // yaw for determining rotation
    return new Pose2d(t, r);
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
  //From -12 to 12V
  public void setWheelVoltages(double frontLeft, double frontRight, double backLeft, double backRight){
    frontLeftMotor.setVoltage(frontLeft);
    frontRightMotor.setVoltage(frontRight);
    backLeftMotor.setVoltage(backLeft);
    backRightMotor.setVoltage(backRight);
  }

//NEEDS TO BE FINALIZED.
//Should be done using velocity PID. Cannot be done without it.
//Sets individual wheel speeds to the mecanum drive.
  public void setKinematicWheelSpeeds(MecanumDriveWheelSpeeds kinematicWheelSpeeds){
    double[] wheelSpeeds = new double[4];
    MecanumDriveWheelSpeeds currentWheelSpeeds = getKinematicWheelSpeeds();

    wheelSpeeds[0] =
      frontLeftMotorPID.calculate(currentWheelSpeeds.frontLeftMetersPerSecond, kinematicWheelSpeeds.frontLeftMetersPerSecond*2.0)
      + frontLeftMotorFF.calculate(kinematicWheelSpeeds.frontLeftMetersPerSecond);
    wheelSpeeds[1] = 
      frontRightMotorPID.calculate(currentWheelSpeeds.frontRightMetersPerSecond, kinematicWheelSpeeds.frontRightMetersPerSecond*2.0)
      + frontRightMotorFF.calculate(kinematicWheelSpeeds.frontRightMetersPerSecond);
    wheelSpeeds[2] =
      backLeftMotorPID.calculate(currentWheelSpeeds.rearLeftMetersPerSecond, kinematicWheelSpeeds.rearLeftMetersPerSecond*2.0)
      + backLeftMotorFF.calculate(kinematicWheelSpeeds.rearLeftMetersPerSecond);
    wheelSpeeds[3] =
      backRightMotorPID.calculate(currentWheelSpeeds.rearRightMetersPerSecond, kinematicWheelSpeeds.rearRightMetersPerSecond*2.0)
      + backRightMotorFF.calculate(kinematicWheelSpeeds.rearRightMetersPerSecond);

    // for (var i = 0; i < wheelSpeeds.length; i++) {
    //   if (wheelSpeeds[i] > 8) {
    //     wheelSpeeds[i] = 8;
    //   } else if (wheelSpeeds[i] < -8) {
    //     wheelSpeeds[i] = -8;
    //   }
    // }

    setWheelVoltages(wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3]);
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
