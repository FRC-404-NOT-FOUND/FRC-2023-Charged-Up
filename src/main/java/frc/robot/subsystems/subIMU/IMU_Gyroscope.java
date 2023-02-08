// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.subIMU;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IMU_Gyroscope extends SubsystemBase implements Gyro {

  SerialPort arduinoTeensy;

  /** Creates a new Gyro. */
  public IMU_Gyroscope(SerialPort serialPort) {
    arduinoTeensy = serialPort;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void close() throws Exception {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void calibrate() {
    // TODO Auto-generated method stub
    arduinoTeensy.writeString("gC");
  }

  @Override
  public void reset() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public double getAngle() {
    // TODO Auto-generated method stub
    getZ();
    return 0;
  }

  @Override
  public double getRate() {
    // TODO Auto-generated method stub
    return 0;
  }

  //Get ALL rotation values.
  public double[] getRotation(){
    arduinoTeensy.writeString("gE");

    String x = arduinoTeensy.readString();
    String y = arduinoTeensy.readString();
    String z = arduinoTeensy.readString();

    double X = Double.parseDouble(x);
    double Y = Double.parseDouble(y);
    double Z = Double.parseDouble(z);

    double[] array = {X, Y, Z};
    
    return array;
  }
  public double getX(){
    arduinoTeensy.writeString("gX");
    return 0;
  }
  public double getY(){
    arduinoTeensy.writeString("gY");
    return 0;
  }
  public double getZ(){
    arduinoTeensy.writeString("gZ");
    return 0;
  }
}
