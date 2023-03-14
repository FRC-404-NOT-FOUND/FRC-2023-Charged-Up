package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.TestDrivetrainPID;
import frc.robot.commands.TestPathPID;
import frc.robot.subsystems.IMU;

public class Robot extends TimedRobot {
  // private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private IMU imu;
  private Command m_autonomousCommand;
  private Command testCommand;

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
    testCommand = new TestDrivetrainPID(m_robotContainer.getDrivetrain());
    PathPlannerServer.startServer(8888);
    m_robotContainer.getGrabber().turnCompressorOn();
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

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    CommandScheduler.getInstance().enable();
    System.out.println("Test Init!");
    m_robotContainer.getGrabber().getHopper().getCompressor().disable();
    if (testCommand != null) {
      System.out.println("Schduling Test Command");
      testCommand.schedule();
    }
    
  }

  @Override
  public void testPeriodic() {
    var kinVel = m_robotContainer.getDrivetrain().getKinematicWheelSpeeds();
    SmartDashboard.putNumber("Front Left Encoder Meters/Second", kinVel.frontLeftMetersPerSecond);
    SmartDashboard.putNumber("Front Right Encoder Meters/Second", kinVel.frontRightMetersPerSecond);
    SmartDashboard.putNumber("Back Left Encoder Meters/Second", kinVel.rearLeftMetersPerSecond);
    SmartDashboard.putNumber("Back Right Encoder Meters/Second", kinVel.rearRightMetersPerSecond);
    CommandScheduler.getInstance().run();
  }
}