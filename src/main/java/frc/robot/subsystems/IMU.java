// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.ADXL362.AllAxes;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.subIMU.Accelerometer;
import frc.robot.subsystems.subIMU.Gyroscope;
import frc.robot.subsystems.subIMU.IMU_Gyroscope;

public class IMU extends SubsystemBase {
  private boolean isActive;
  private boolean isReady = false;
  private SerialPort arduinoTeensy;
  private IMU_Gyroscope imu_gyro;
  private Accelerometer accel;
  private Gyroscope gyro;
  private static IMU instance = null;

  public static IMU create() {
    if (instance == null) {
      instance = new IMU();
    }

    return instance;
  }

  public void connect() {
    isActive = connectSerialPort();
    gyro = new Gyroscope();
    accel = new Accelerometer();

    if(isActive){
      imu_gyro = new IMU_Gyroscope(arduinoTeensy);

      while(!isReady){
        if (arduinoTeensy.getBytesReceived() > 0) {
          String data = arduinoTeensy.readString();
          System.out.println(data);

          if(data.contains("READY")){
            System.out.println("IMU IS READY!");
            isReady = true;
            break;
          }
        }
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(getGyroPitch());
  }

  //Connects to, and sets up the serial port
  public boolean connectSerialPort(){
    try{
      arduinoTeensy = new SerialPort(Constants.SERIAL_BAUD_RATE, SerialPort.Port.kUSB);
      System.out.println("Connected to Serial Port kUSB");

      return true;
    }
    catch(Exception e){
      System.out.println("Failed to Connect to kUSB");
      try{
        arduinoTeensy = new SerialPort(Constants.SERIAL_BAUD_RATE, SerialPort.Port.kUSB1);
        System.out.println("Connected to Serial Port kUSB1");

        return true;
      }
      catch(Exception e1){
        System.out.println("Failed to Connect to kUSB1");
        try{
          arduinoTeensy = new SerialPort(Constants.SERIAL_BAUD_RATE, SerialPort.Port.kUSB2);
          System.out.println("Connected to Serial Port kUSB2");    
          return true;
        }
        catch(Exception e2){
          System.out.println("Failed to Connect to kUSB2, all USB connections failed.");
        }
      }
    }
    return false;
  }

  public boolean isIMUActive(){
    return isActive;
  }

  public boolean isIMUReady(){
    return isReady;
  }

  //gets all angles
  public double[] getGyroRotation(){
    double[] arr = {getGyroYaw(), getGyroPitch(), getGyroRoll()}; // we still want the yaw from the ADXRS450 gyro
    return arr;
  }

  public double getGyroYaw(){
    return gyro.getYaw();
  }
  public double getGyroPitch(){
    return imu_gyro.getPitch();
  }
  public double getGyroRoll(){
    return imu_gyro.getRoll();
  }

  //Gets ALL Acceleration
  public AllAxes getAcceleration(){
    return accel.getAccelerations();
  }
  public double getAccelX(){
    return accel.getX();
  }
  public double getAccelY(){
    return accel.getY();
  }
  public double getAccelZ(){
    return accel.getZ();
  }

  public void calibrate(){
    imu_gyro.calibrate();
    gyro.calibrate();
  }
}
