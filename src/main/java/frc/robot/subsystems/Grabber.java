// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.subGrabber.G_Hopper;
import frc.robot.subsystems.subGrabber.G_Intake;

public class Grabber extends SubsystemBase {
  /** Creates a new Grabber. */
  private G_Hopper hopper;
  private G_Intake intake;
  private final double CUBE_HOLD_VOLTAGE = 0.5;
  private final double CONE_HOLD_VOLTAGE = 0.75;

  private boolean hasCone;
  private boolean hasCube;
  
  public Grabber(boolean cone, boolean cube) {
    hopper = new G_Hopper();
    intake = new G_Intake();

    hasCone = cone;
    hasCube = cube;
  }

  public Grabber() {
    this(false, false);
  }

  @Override
  public void periodic() {}

  public G_Hopper getHopper(){
    return hopper;
  }

  public G_Intake getIntake(){
    return intake;
  }

  public Command intakeCubeCommand(Arm arm) {
    Debouncer debounce = new Debouncer(0.75, DebounceType.kRising);

    return Commands.either(
      Commands.runOnce(() -> {
        debounce.calculate(false);
        hopper.pneumaticsOpen();
      })
        .andThen(intakeCommand(0.4).until(() -> debounce.calculate(intake.hasSpiked())))
        .finallyDo((interruped) -> {
          hasCube = true;
          intake.main.setVoltage(CUBE_HOLD_VOLTAGE);
        }),
      Commands.runOnce(() -> {}),
      () -> !hasCube && !hasCone
    );
  }

  public Command intakeConeCommand(Arm arm) {
    Debouncer debounce = new Debouncer(0.75, DebounceType.kRising);

    return Commands.either(
      Commands.runOnce(() -> {
        debounce.calculate(false);
        hopper.pneumaticsClose();
      })
        .andThen(intakeCommand(0.4).until(() -> debounce.calculate(intake.hasSpiked())))
        .finallyDo((interruped) -> {
          hasCone = true;
          intake.main.setVoltage(CONE_HOLD_VOLTAGE);
        }),
      Commands.runOnce(() -> {}),
      () -> !hasCone && !hasCube
    );
  }

  public Command ejectConeCommand() {
    return Commands.either(
      runOnce(() -> {
        intake.stop();
        hopper.pneumaticsOpen();
        hasCone = false;
      }),
      Commands.runOnce(() -> {}),
      () -> hasCone
    );
  }

  public Command ejectCubeCommand() {
    Timer timer = new Timer();

    return Commands.either(
      runOnce(() -> {
        timer.reset();
        timer.start();
      })
        .andThen(spitCommand(0.8).until(() -> timer.hasElapsed(0.5)))
        .finallyDo((interupped) -> {
          timer.stop();
          intake.stop();
          hasCube = false;
        }),
      Commands.runOnce(() -> {}),
      () -> hasCube
    );
  }

  public Command toggleConeCommand(Arm arm) {
    return Commands.either(ejectConeCommand(), intakeConeCommand(arm), () -> hasCone);
      //.andThen(arm.extendArmTo(0));
  }

  public Command toggleCubeCommand(Arm arm) {
    return Commands.either(ejectCubeCommand(), intakeCubeCommand(arm), () -> hasCube);
      //.andThen(arm.extendArmTo(0));
  }

  public Command intakeCommand(double speed) {
    return intake.intakeCommand(speed);
  }

  public Command spitCommand(double speed) {
    return intake.spitCommand(speed);
  }

  public Command toggleGrabberCommand() {
    return hopper.toggleGrabberCommand();
  }

  public void turnCompressorOn() {
    hopper.turnCompressorOn();
  }

  public void turnCompressorOff() {
    hopper.turnCompressorOff();
  }
}
