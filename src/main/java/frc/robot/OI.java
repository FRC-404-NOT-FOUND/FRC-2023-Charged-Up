package frc.robot;

import edu.wpi.first.wpilibj.*;

public class OI {
    private final XboxController gamepad = new XboxController(Constants.DRIVERSTATION_CONTROL_PORT);

    public double getDriverRawAxis(int axis){
        return gamepad.getRawAxis(axis);
    }

}
