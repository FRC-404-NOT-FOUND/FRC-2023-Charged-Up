// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI.Port;

public final class Constants {
    public static Timer timer = new Timer();
    public static double aprilTagMoveX = 0;
    public static double aprilTagMoveY = 0;
    public static double aprilTagRotate;

//Drivetrain
    //PWM
    public static final int FRONT_LEFT_MOTOR_INDEX = 8;
    public static final int FRONT_RIGHT_MOTOR_INDEX = 1;
    public static final int BACK_LEFT_MOTOR_INDEX = 6;
    public static final int BACK_RIGHT_MOTOR_INDEX = 0;

    //CAN
    public static final int FRONT_LEFT_ENCODER_INDEX = 31;
    public static final int FRONT_RIGHT_ENCODER_INDEX = 32;
    public static final int BACK_LEFT_ENCODER_INDEX = 33;
    public static final int BACK_RIGHT_ENCODER_INDEX = 34;

    //Motor Velocity PID (Proportional, Integral, Derivative)
    public static final double FRONT_LEFT_MOTOR_KP = 2.75;
    public static final double FRONT_LEFT_MOTOR_KI = 0.0;
    public static final double FRONT_LEFT_MOTOR_KD = 0.258;
    public static final double FRONT_RIGHT_MOTOR_KP = 2.79;
    public static final double FRONT_RIGHT_MOTOR_KI = 0.0;
    public static final double FRONT_RIGHT_MOTOR_KD = 0.25;
    public static final double BACK_LEFT_MOTOR_KP = 2.695;
    public static final double BACK_LEFT_MOTOR_KI = 0.0;
    public static final double BACK_LEFT_MOTOR_KD = 0.20;
    public static final double BACK_RIGHT_MOTOR_KP = 2.73;
    public static final double BACK_RIGHT_MOTOR_KI = 0.0;
    public static final double BACK_RIGHT_MOTOR_KD = 0.23;
    
    //Motor Velocity Feedforward (Static Friction, Velocity-keepup)
    //More Info:
    //  https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-feedforward.html#the-permanent-magnet-dc-motor-feedforward-equation
    public static final double DRIVETRAIN_FF_KS = 0.15;
    public static final double DRIVETRAIN_FF_KV = 3.18;
    public static final double DRIVETRAIN_FF_KA = 0.0;

    //General Drivetrain PID (from Cartesian movement)
    public static final double DRIVETRAIN_TRANSFORM_KPx = 0.1;
    public static final double DRIVETRAIN_TRANSFORM_KIx = 0.0;
    public static final double DRIVETRAIN_TRANSFORM_KDx = 0.001;
    public static final double DRIVETRAIN_TRANSFORM_KPy = 0.8; //This may have to be negative to make it go towards the horizontal "line"
    public static final double DRIVETRAIN_TRANSFORM_KIy = 0.0;
    public static final double DRIVETRAIN_TRANSFORM_KDy = 1;
    public static final double DRIVETRAIN_ROTATE_KP = 0.1;
    public static final double DRIVETRAIN_ROTATE_KI = 0.0;
    public static final double DRIVETRAIN_ROTATE_KD = 0.01;
    public static final double MAX_AUTONOMOUS_WHEEL_VELOCITY = 2.0; //In Meters per Second
    public static final double MAX_AUTONOMOUS_WHEEL_ACCEL = 1.0;

    //radius in meters
    public static final double DRIVETRAIN_WHEEL_RADIUS = 0.1016; //4 inches
    
    //Find the radius of the wheel, multiply by 2*pi*r
    public static final double ROTATIONS_TO_METERS = 2.0 * Math.PI * Constants.DRIVETRAIN_WHEEL_RADIUS;
    public static final double DEGREES_TO_METERS = (1.0/360.0) * ROTATIONS_TO_METERS;

    //Motor Speed Offsets
    public static final double FRONT_LEFT_MOTOR_SPEED_OFFSET = 0.60;
    public static final double FRONT_RIGHT_MOTOR_SPEED_OFFSET = 0.55;
    public static final double BACK_LEFT_MOTOR_SPEED_OFFSET = 1.0;
    public static final double BACK_RIGHT_MOTOR_SPEED_OFFSET = 0.60;

    //Fine Control Constant
    public static final double DRIVETRAIN_FINE_CONTROL = 0.25;

    //Kinematics
    public static Translation2d FRONT_LEFT_WHEEL_TO_CENTER = new Translation2d(0.259, 0.262);
    public static Translation2d FRONT_RIGHT_WHEEL_TO_CENTER = new Translation2d(0.259, -0.262);
    public static Translation2d BACK_LEFT_WHEEL_TO_CENTER = new Translation2d(-0.259, 0.262);
    public static Translation2d BACK_RIGHT_WHEEL_TO_CENTER= new Translation2d(-0.259, -0.262);

    //IMU
    public static final int SERIAL_BAUD_RATE = 9600;

    //Analog Gyro/Accel
    public static final Port ADXRS450_GYRO_PORT = SPI.Port.kOnboardCS0;

    //Grabber_Intake
    public static final int INTAKE_LEFT_MOTOR_INDEX = 21;
    public static final int INTAKE_RIGHT_MOTOR_INDEX = 22;
    
    //Grabber_Hopper (the pneumatics)
    public static final int HOPPER_PNEUMATIC_FWD_PORT = 0;
    public static final int HOPPER_PNEUMATIC_REV_PORT = 1;

    //Arm_Extension
    public static final int EXTENSION_WHEEL_MOTOR_INDEX = 11;
    public static final int EXTENSION_WHEEL_MAX_POSITION = 220;
    public static final int EXTENSION_WHEEL_MIN_POSITION = 0;

    //Arm_Pivot
    public static final int PIVOT_MOTOR_INDEX = 1;
    public static final int PIVOT_SLOW_PID_SLOT = 0;
    public static final int PIVOT_QUICK_PID_SLOT = 1;
    public static final int PIVOT_MIN_POSITION = 100;
    public static final int PIVOT_MAX_POSITION = 500;

    //Positions
    public static final int FIRST_CONE_ANGLE = 126;
    public static final int FIRST_CONE_EXTENSION = 42;
    public static final int SECOND_CONE_ANGLE = 142;
    public static final int SECOND_CONE_EXTENSION = 200;
    public static final int FIRST_CUBE_ANGLE = 95;
    public static final int FIRST_CUBE_EXTENSION = 25;
    public static final int SECOND_CUBE_ANGLE = 130;
    public static final int SECOND_CUBE_EXTENSION = 160;
    public static final int DEFAULT_ANGLE = 32;
    public static final int DEFAULT_EXTENSION = 50;


    //OI
    public static final int DRIVER_LEFT_CONTROL_PORT = 1;
    public static final int DRIVER_RIGHT_CONTROL_PORT= 2;
    public static final int DRIVER_GAMEPAD_CONTROL_PORT = 0;

    public static final int GAMEPAD_LEFT_STICK_X = 0;
    public static final int GAMEPAD_LEFT_STICK_Y = 1;
    public static final int GAMEPAD_RIGHT_STICK_X = 4;
    public static final double GAMEPAD_DEADZONE = 0.1;
}
