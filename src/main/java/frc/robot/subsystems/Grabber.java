// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.subGrabber.G_Hopper;
import frc.robot.subsystems.subGrabber.G_Intake;

public class Grabber extends SubsystemBase {
  /** Creates a new Grabber. */
  private G_Hopper hopper;
  private G_Intake intake;

  private final double STALL_CURRENT = 4;
  private final double CUBE_HOLD_SPEED = 0.05;

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

  public Command intakeCubeCommand() {
    Debouncer debounce = new Debouncer(0.5, DebounceType.kRising);

    return Commands.either(
      runOnce(() -> {
        debounce.calculate(false);
        hopper.pneumaticsOpen();
      })
        .andThen(intakeCommand(0.4).until(() -> debounce.calculate(intake.getFilteredCurrent() > STALL_CURRENT)))
        .finallyDo((interruped) -> {
          hasCube = true;
          intake.start(CUBE_HOLD_SPEED);
        }),
      Commands.runOnce(() -> {}),
      () -> !hasCube && !hasCone
    );
  }

  public Command intakeConeCommand() {
    Debouncer debounce = new Debouncer(0.5, DebounceType.kRising);

    return Commands.either(
      runOnce(() -> {
        debounce.calculate(false);
        hopper.pneumaticsClose();
      })
        .andThen(intakeCommand(0.4).until(() -> debounce.calculate(intake.getFilteredCurrent() > STALL_CURRENT)))
        .finallyDo((interruped) -> hasCone = true),
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
    return Commands.either(
      spitCommand(0.1)
        .andThen(Commands.waitSeconds(0.5))
        .finallyDo((interupped) -> {
          intake.stop();
          hasCube = false;
        }),
      Commands.runOnce(() -> {}),
      () -> hasCube
    );
  }

  public Command toggleConeCommand() {
    return Commands.either(ejectConeCommand(), intakeConeCommand(), () -> hasCone);
  }

  public Command toggleCubeCommand() {
    return Commands.either(ejectCubeCommand(), intakeCubeCommand(), () -> hasCube);
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
