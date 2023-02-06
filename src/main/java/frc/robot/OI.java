package frc.robot;

import edu.wpi.first.wpilibj.*;

public class OI {
    public static Joystick driverLeft = new Joystick(Constants.DRIVER_LEFT_CONTROL_PORT);
    public static Joystick driverRight = new Joystick(Constants.DRIVER_RIGHT_CONTROL_PORT);

    public static XboxController gamepad = new XboxController(Constants.DRIVER_GAMEPAD_CONTROL_PORT);
}
