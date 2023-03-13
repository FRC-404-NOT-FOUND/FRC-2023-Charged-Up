// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Returns true if the pneumatics system is closed
  public boolean pneumaticsCloseState(){
    return hopper.pneumaticsCloseState();
  }

  //Returns true if the pneumatics system is open
  public boolean pneumaticsOpenState(){
    return hopper.pneumaticsOpenState();
  }

  public G_Hopper getHopper(){
    return hopper;
  }

  public G_Intake getIntake(){
    return intake;
  }

  public void startIntake() {
    intake.start(0.4);
  }

  public void stopIntake() {
    intake.stop();
  }

  public void startSpit() {
    intake.spit(0.1);
  }

  public Command intakeCubeCommand() {
    Debouncer debounce = new Debouncer(1, DebounceType.kRising);

    return runOnce(() -> {
        debounce.calculate(false);
        hopper.pneumaticsOpen();
      })
        .andThen(intakeCommand(0.4).until(() -> debounce.calculate(intake.getFilteredCurrent() > STALL_CURRENT)))
        .finallyDo((interruped) -> {
          hasCube = true;
          intake.start(CUBE_HOLD_SPEED);
        });
  }

  public Command intakeConeCommand() {
    Debouncer debounce = new Debouncer(1, DebounceType.kRising);

    return runOnce(() -> {
        debounce.calculate(false);
        hopper.pneumaticsClose();
      })
        .andThen(intakeCommand(0.4).until(() -> debounce.calculate(intake.getFilteredCurrent() > STALL_CURRENT)))
        .finallyDo((interruped) -> hasCone = true);
  }

  public Command ejectConeCommand() {
    return runOnce(() -> {
      intake.stop();
      hopper.pneumaticsOpen();
      hasCone = false;
    });
  }

  public Command ejectCubeCommand() {
    return runOnce(() -> intake.spit(0.1))
      .andThen(Commands.waitSeconds(0.5))
      .finallyDo((interupped) -> {
        intake.stop();
        hasCube = false;
      });
  }

  public Command toggleConeCommand() {
    return new ConditionalCommand(ejectConeCommand(), intakeConeCommand(), () -> hasCone);
  }

  public Command toggleCubeCommand() {
    return new ConditionalCommand(ejectCubeCommand(), intakeCubeCommand(), () -> hasCube);
  }

  public Command intakeCommand(double speed) {
    return intake.intakeCommand(speed);
  }

  public Command spitCommand(double speed) {
    return intake.spitCommand(speed);
  }

  public Command toggleCompressorCommand() {
    return hopper.toggleCompressorCommand();
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
