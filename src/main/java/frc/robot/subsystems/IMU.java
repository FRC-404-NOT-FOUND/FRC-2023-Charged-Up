// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.subIMU.IMU_Accelerometer;
import frc.robot.subsystems.subIMU.IMU_Gyroscope;

public class IMU extends SubsystemBase {
  private boolean isActive;
  private SerialPort arduinoTeensy;
  private IMU_Gyroscope imu_gyro;
  private IMU_Accelerometer imu_accel;

  /** Creates a new IMU. */
  public IMU() {
    isActive = connectSerialPort();

    if(isActive){
      imu_gyro = new IMU_Gyroscope(arduinoTeensy);
      imu_accel = new IMU_Accelerometer(arduinoTeensy);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

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

  //gets all angles
  public double[] getGyroRotation(){
    return imu_gyro.getRotation();
  }
  public double getGyroX(){
    return imu_gyro.getX();
  }
  public double getGyroY(){
    return imu_gyro.getY();
  }
  public double getGyroZ(){
    return imu_gyro.getZ();
  }

  //Gets ALL Acceleration
  public double[] getAcceleration(){
    return imu_accel.getAcceleration();
  }
  public double getAccelX(){
    return imu_accel.getX();
  }
  public double getAccelY(){
    return imu_accel.getY();
  }
  public double getAccelZ(){
    return imu_accel.getZ();
  }

  public void calibrate(){
    imu_accel.calibrate();
    imu_gyro.calibrate();
  }
}
