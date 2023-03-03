// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;

public final class Constants {
    public static Timer timer = new Timer();
    public static double aprilTagMoveHorizontal = 0;
    public static double aprilTagMoveVertical = 0;

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

    //Mass of the Robot
    public static final double ROBOT_MASS_KG = 110;

    //radius in meters
    public static final double DRIVETRAIN_WHEEL_RADIUS = 0.1016; //4 inches
    
    //Find the radius of the wheel, multiply by 2*pi*r
    public static final double ROTATIONS_TO_METERS = 2 * Math.PI * Constants.DRIVETRAIN_WHEEL_RADIUS;

    //Motor Speed Offsets
    public static final double FRONT_LEFT_MOTOR_SPEED_OFFSET = 0.60;
    public static final double FRONT_RIGHT_MOTOR_SPEED_OFFSET = 0.55;
    public static final double BACK_LEFT_MOTOR_SPEED_OFFSET = 1.0;
    public static final double BACK_RIGHT_MOTOR_SPEED_OFFSET = 0.60;

    //Kinematics (THESE WILL NEED TO BE DEFINED)
    public static Translation2d FRONT_LEFT_WHEEL_TO_CENTER = new Translation2d(0.259, 0.262);
    public static Translation2d FRONT_RIGHT_WHEEL_TO_CENTER = new Translation2d(0.259, -0.262);;
    public static Translation2d BACK_LEFT_WHEEL_TO_CENTER = new Translation2d(-0.259, 0.262);;
    public static Translation2d BACK_RIGHT_WHEEL_TO_CENTER= new Translation2d(-0.259, -0.262);;

    //IMU
    public static final int SERIAL_BAUD_RATE = 9600;

    //Misc.
    public static final double DRIVETRAIN_TRANSFORM_KP = 0.0;
    public static final double DRIVETRAIN_TRANSFORM_KI = 0.0;
    public static final double DRIVETRAIN_TRANSFORM_KD = 0.0;
    public static final double DRIVETRAIN_ROTATE_KP = 0.0;
    public static final double DRIVETRAIN_ROTATE_KI = 0.0;
    public static final double DRIVETRAIN_ROTATE_KD = 0.0;
    public static final double MAX_AUTONOMOUS_WHEEL_SPEED = 0.0; //In Meters per Second
    public static final double MAX_AUTONOMOUS_WHEEL_ACCEL = 0.0;

    //AprilTag PID
    public static final double APRIL_TAG_P = 1;
    public static final double APRIL_TAG_I = 0.0;
    public static final double APRIL_TAG_D = 0.00;

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
    public static final int FIRST_CONE_ANGLE = 320;
    public static final int FIRST_CONE_EXTENSION = 42;
    public static final int SECOND_CONE_ANGLE = 405;
    public static final int SECOND_CONE_EXTENSION = 200;
    public static final int FIRST_CUBE_ANGLE = 255;
    public static final int FIRST_CUBE_EXTENSION = 25;
    public static final int SECOND_CUBE_ANGLE = 355;
    public static final int SECOND_CUBE_EXTENSION = 160;
    public static final int DEFAULT_ANGLE = 100;
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
