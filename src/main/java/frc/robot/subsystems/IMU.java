// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.subIMU.IMU_Gyroscope;

public class IMU extends SubsystemBase {
  private boolean arduinoConnected = false;
  private boolean devicesConnected = false;
  private boolean isReady = false;
  private SerialPort arduinoTeensy;
  private IMU_Gyroscope imu_gyro;
  private BuiltInAccelerometer accel;
  private ADXRS450_Gyro gyro;
  private double filteredRate = 0;
  private double filteredHeading = 0;
  private LinearFilter rateFilter = LinearFilter.movingAverage(20);
  private LinearFilter headingFilter = LinearFilter.movingAverage(20);
  private static IMU instance = null;

  public static IMU create() {
    if (instance == null) {
      instance = new IMU();
    }

    return instance;
  }

  private IMU(){}

  @Override
  public void periodic() {
    //System.out.println("Gyro ready: " + isGyroReady());
    
    //TODO: Make this work after comp this weekend.
    // if (arduinoConnected && !isReady) {
    //   readyArduino();
    // } else if (!arduinoConnected) {
    //   connectArduino();
    // }

    if (isGyroReady()) {
      filteredRate = rateFilter.calculate(MathUtil.applyDeadband(gyro.getRate(), 0.05));
      filteredHeading = headingFilter.calculate(MathUtil.applyDeadband(gyro.getAngle(), 0.05));
    }
  }

  public void readyArduino() {
    if(arduinoConnected && !isReady){
      //NOTE: Must make this null in order for Simulations to work.
      String cache = "";
      if (arduinoTeensy.getBytesReceived() > 0) {
        String data = arduinoTeensy.readString();
        System.out.println(data);
        cache.concat(data.trim());

        if(cache.contains("READY")){
          System.out.println("IMU IS READY!");
          imu_gyro = new IMU_Gyroscope(arduinoTeensy);
          isReady = true;
          calibrateArduino();
        }
      }
    }
  }

  public void connectDevices() {
    gyro = new ADXRS450_Gyro();
    calibrateGyro();
    accel = new BuiltInAccelerometer(Range.k2G);
    devicesConnected = true;
  }

  //Connects to, and sets up the serial port
  public void connectArduino(){
    try{
      arduinoTeensy = new SerialPort(Constants.SERIAL_BAUD_RATE, SerialPort.Port.kUSB);
      System.out.println("Connected to Serial Port kUSB");

      arduinoConnected = true;
    }
    catch(Exception e){
      System.out.println("Failed to Connect to kUSB");
      try{
        arduinoTeensy = new SerialPort(Constants.SERIAL_BAUD_RATE, SerialPort.Port.kUSB1);
        System.out.println("Connected to Serial Port kUSB1");

        arduinoConnected = true;
      }
      catch(Exception e1){
        System.out.println("Failed to Connect to kUSB1");
        try{
          arduinoTeensy = new SerialPort(Constants.SERIAL_BAUD_RATE, SerialPort.Port.kUSB2);
          System.out.println("Connected to Serial Port kUSB2");    
          
          arduinoConnected = true;
        }
        catch(Exception e2){
          System.out.println("Failed to Connect to kUSB2, all USB connections failed.");
        }
      }
    }
  }

  public Command reconnectAruino() {
    return runOnce(() -> {
      if(!isArduinoConnected()){
        connectArduino();
      } else {
        System.out.println("Arduino is already connected dumbo.");
      }
    });
  }

  public boolean isArduinoConnected(){
    return arduinoConnected;
  }

  public boolean areDevicesConnected(){
    return devicesConnected;
  }

  public boolean isIMUReady(){
    return isReady;
  }

  public boolean isGyroReady(){
    return devicesConnected && gyro.isConnected();
  }

  public double getGyroRate() {
    if (isGyroReady()) {
      return filteredRate;
    }

    return 0.0;
  }

  public double getGyroYaw(){
    if (isGyroReady()) {
      return filteredHeading;
    }

    return 0.0;
  }
  public double getGyroPitch(){
    return imu_gyro.getPitch();
  }
  public double getGyroRoll(){
    return imu_gyro.getRoll();
  }

  //Gets ALL Acceleration
  public double getAccelX(){
    return accel.getX();
  }
  public double getAccelY(){
    return accel.getY();
  }
  public double getAccelZ(){
    return accel.getZ();
  }

  public void calibrateArduino(){
    imu_gyro.calibrate();
  }

  public void calibrateGyro() {
    gyro.calibrate();
  }
}
