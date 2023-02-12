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
    System.out.println(arduinoTeensy.readString()); //Calibrating IMU...
    String done = arduinoTeensy.readString();
    while(done == ""){
      done = arduinoTeensy.readString();
    }
    System.out.println(arduinoTeensy.readString()); //DONE!
  }

  @Override
  public void reset() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public double getAngle() {
    // TODO Auto-generated method stub
    getYaw();
    return 0;
  }

  @Override
  public double getRate() {
    // TODO Auto-generated method stub
    return 0;
  }

  //Get ALL rotation values.
  public double[] getRotation(){
    arduinoTeensy.writeString("gA");
    while(arduinoTeensy.getBytesReceived() == 0){}

    double y = Double.parseDouble(arduinoTeensy.readString());
    double p = Double.parseDouble(arduinoTeensy.readString());
    double r = Double.parseDouble(arduinoTeensy.readString());

    double[] array = {y, p, r};
    
    return array;
  }
  public double getYaw(){
    arduinoTeensy.writeString("gY");

    while(arduinoTeensy.getBytesReceived() == 0){}

    int availableBytes = arduinoTeensy.getBytesReceived();
    System.out.println(availableBytes);
    boolean messageIsReady = false;

    while (!messageIsReady){
      byte[] mes = arduinoTeensy.read(availableBytes);
      String message = new String(mes);

      try {
        double yaw = Double.parseDouble(message);
        System.out.print("YAW: ");
        System.out.println(message);
        return yaw;
      } 
      catch (Exception e) {
        return 0.0;
      }
    }
    //This shouldn't happen
    return 0.0;
  }
  public double getPitch(){
    arduinoTeensy.writeString("gP");
    return Double.parseDouble(arduinoTeensy.readString());
  }
  public double getRoll(){
    arduinoTeensy.writeString("gR");
    return Double.parseDouble(arduinoTeensy.readString());
  }
}
