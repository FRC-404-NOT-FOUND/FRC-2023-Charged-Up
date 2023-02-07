// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {

//Drivetrain
    public static final int FRONT_LEFT_MOTOR_INDEX = 2;
    public static final int FRONT_RIGHT_MOTOR_INDEX = 3;
    public static final int BACK_LEFT_MOTOR_INDEX = 1;
    public static final int BACK_RIGHT_MOTOR_INDEX = 0;

    //THESE WILL NEED TO BE DEFINED
    public static Translation2d FRONT_LEFT_WHEEL_TO_CENTER;
    public static Translation2d FRONT_RIGHT_WHEEL_TO_CENTER;
    public static Translation2d BACK_LEFT_WHEEL_TO_CENTER;
    public static Translation2d BACK_RIGHT_WHEEL_TO_CENTER;

    public static final double MAX_AUTONOMOUS_WHEEL_SPEED = 0.0; //In Meters per Second

//Grabber_Intake
    public static final int INTAKE_LEFT_MOTOR_INDEX = 0;
    public static final int INTAKE_RIGHT_MOTOR_INDEX = 0;
    
//Grabber_Hopper (the pneumatics)
    public static final int HOPPER_PNEUMATIC_FWD_PORT = 0;
    public static final int HOPPER_PNEUMATIC_REV_PORT = 1;

//Arm_Extension
    public static final int EXTENSION_WHEEL_MOTOR_INDEX = 0;

//Arm_Pivot
    public static final int PIVOT_MOTOR_INDEX = 0;

//OI
    public static final int DRIVER_LEFT_CONTROL_PORT = 0;
    public static final int DRIVER_RIGHT_CONTROL_PORT= 1;
    public static final int DRIVER_GAMEPAD_CONTROL_PORT = 2;

    public static final int GAMEPAD_LEFT_STICK_X = 0;
    public static final int GAMEPAD_LEFT_STICK_Y = 1;
    public static final int GAMEPAD_RIGHT_STICK_X = 4;
    public static final double GAMEPAD_DEADZONE = 0.1;
}
