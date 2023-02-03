package frc.robot;

import edu.wpi.first.wpilibj.*;

public class OI {
    private final Joystick driverLeft = new Joystick(Constants.DRIVER_LEFT_CONTROL_PORT);
    private final Joystick driverRight = new Joystick(Constants.DRIVER_RIGHT_CONTROL_PORT);


    private final XboxController gamepad = new XboxController(Constants.DRIVERSTATION_CONTROL_PORT);

    public double getDriverRawAxis(int axis){
        return gamepad.getRawAxis(axis);
    }

}
