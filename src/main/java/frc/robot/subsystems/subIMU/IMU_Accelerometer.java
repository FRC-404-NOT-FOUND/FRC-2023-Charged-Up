// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.subIMU;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IMU_Accelerometer extends SubsystemBase implements Accelerometer{

  SerialPort arduinoTeensy;

  /** Creates a new Accel. */
  public IMU_Accelerometer(SerialPort serialPort) {
    arduinoTeensy = serialPort;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void setRange(Range range) {
    // TODO Auto-generated method stub
    
  }

  public double[] getAcceleration(){
    arduinoTeensy.writeString("aA");

    String x = arduinoTeensy.readString();
    String y = arduinoTeensy.readString();
    String z = arduinoTeensy.readString();

    double X = Double.parseDouble(x);
    double Y = Double.parseDouble(y);
    double Z = Double.parseDouble(z);

    double[] array = {X, Y, Z};
    
    return array;
  }

  @Override
  public double getX() {
    // TODO Auto-generated method stub
    arduinoTeensy.writeString("aX");

    return 0;
  }

  @Override
  public double getY() {
    // TODO Auto-generated method stub
    arduinoTeensy.writeString("aY");
    return 0;
  }

  @Override
  public double getZ() {
    // TODO Auto-generated method stub
    arduinoTeensy.writeString("aZ");
    return 0;
  }

  public void calibrate(){
    arduinoTeensy.writeString("aC");
  }
}
