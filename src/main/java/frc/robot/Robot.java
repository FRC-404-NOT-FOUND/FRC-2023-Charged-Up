package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.subsystems.IMU;

public class Robot extends TimedRobot {
  // private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private IMU imu;
  private Command m_autonomousCommand;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    //TODO: FIDDLE WITH ARDUINO AFTER COMP THIS WEEKEND
    imu = IMU.create();
    imu.connectDevices();
    //imu.connectArduino();
    m_robotContainer = new RobotContainer();
    PathPlannerServer.startServer(8888);
    System.out.println("Robot Inited");
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /**  This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    Constants.timer.start();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
  }
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
  }

public ShuffleboardTab tab = Shuffleboard.getTab("PID_DrivetrainTest");
  
public PIDController pid = new PIDController(0, 0, 0);

public PIDController pid_FL = new PIDController(Constants.FRONT_LEFT_MOTOR_KP, 0, Constants.FRONT_LEFT_MOTOR_KD);
public PIDController pid_FR = new PIDController(Constants.FRONT_RIGHT_MOTOR_KP, 0, Constants.FRONT_RIGHT_MOTOR_KD);
public PIDController pid_BL = new PIDController(Constants.BACK_LEFT_MOTOR_KP, 0, Constants.BACK_LEFT_MOTOR_KD);
public PIDController pid_BR = new PIDController(Constants.BACK_RIGHT_MOTOR_KP, 0, Constants.BACK_RIGHT_MOTOR_KD);

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

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.getGrabber().getHopper().getCompressor().disable();
    
  }

  @Override
  public void testPeriodic() {
    var kinVel = m_robotContainer.getDrivetrain().getKinematicWheelSpeeds();
    SmartDashboard.putNumber("Front Left Encoder Meters/Second", kinVel.frontLeftMetersPerSecond);
    SmartDashboard.putNumber("Front Right Encoder Meters/Second", kinVel.frontRightMetersPerSecond);
    SmartDashboard.putNumber("Back Left Encoder Meters/Second", kinVel.rearLeftMetersPerSecond);
    SmartDashboard.putNumber("Back Right Encoder Meters/Second", kinVel.rearRightMetersPerSecond);

    pid.setPID(kP.getDouble(0.0), kI.getDouble(0.0), kD.getDouble(0.0));

    switch((int) Motor.getInteger(0)){
      case(0):
        m_robotContainer.getDrivetrain()
          .setWheelVoltages(pid.calculate(Velocity.getDouble(0.0)), 0.0, 0.0, 0.0);
        break;
      case(1):
        m_robotContainer.getDrivetrain()
            .setWheelVoltages(0.0, pid.calculate(Velocity.getDouble(0.0)), 0.0, 0.0);
        break;
      case(2):
        m_robotContainer.getDrivetrain()
            .setWheelVoltages(0.0, 0.0, pid.calculate(Velocity.getDouble(0.0)), 0.0);
        break;
      case(3):
       m_robotContainer.getDrivetrain()
            .setWheelVoltages(0.0, 0.0, 0.0, pid.calculate(Velocity.getDouble(0.0)));
        break;
      case(4):
        m_robotContainer.getDrivetrain()
          .setWheelVoltages(
            pid.calculate(Velocity.getDouble(0.0)), 
            pid.calculate(Velocity.getDouble(0.0)), 
            pid.calculate(Velocity.getDouble(0.0)), 
            pid.calculate(Velocity.getDouble(0.0))
        );
        break;
      case(5):
        m_robotContainer.getDrivetrain()
          .setWheelVoltages(
            pid_FL.calculate(Velocity.getDouble(0.0)), 
            pid_FR.calculate(Velocity.getDouble(0.0)),  
            pid_BL.calculate(Velocity.getDouble(0.0)), 
            pid_BR.calculate(Velocity.getDouble(0.0))
        );
        break;

      default:
        break;
      
      
    }

  }
}
