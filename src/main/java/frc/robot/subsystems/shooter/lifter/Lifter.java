package frc.robot.subsystems.shooter.lifter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.lifter.LifterConstants.LifterState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Lifter extends SubsystemBase {
  private final LifterIO io;
  private LifterIOInputsAutoLogged inputs = new LifterIOInputsAutoLogged();

  private Double[] setpointsDegs = new Double[3];
  ;
  private LifterState[] lifterStates = new LifterState[3];

  public Lifter(LifterIO lifterIO) {
    this.io = lifterIO;
    for (int i = 0; i < 3; i++) lifterStates[i] = LifterState.STOWED;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Lifter", inputs);
  }

  public boolean hasReachedSetpoint(int lifter) {
    return Math.abs(setpointsDegs[lifter] - inputs.positionDeg[lifter]) < 10;
  }

  public void setState(int lifter, LifterState newState) {
    Double newSetpoint = LifterConstants.SETPOINTS.get(newState);
    if (newSetpoint != null) {
      io.setPosition(lifter, newSetpoint);
      setpointsDegs[lifter] = newSetpoint;
      //   Logger.recordOutput("Lifter/Setpoints", setpointsDegs);
    }
  }

  public void setState(LifterState newState) {
    Double newSetpoint = LifterConstants.SETPOINTS.get(newState);
    if (newSetpoint != null) {
      io.setPosition(newSetpoint);
      for (int i = 0; i < 3; i++) {
        setpointsDegs[i] = newSetpoint;
      }
      //   Logger.recordOutput("Lifter/Setpoints", setpointsDegs);
    }
  }

  public Command raiseAll() {
    return Commands.runOnce(() -> setState(LifterState.RAISED));
  }

  public Command lowerAll() {
    return Commands.runOnce(() -> setState(LifterState.LOWERING));
  }

  public Command stowAll() {
    return Commands.runOnce(() -> setState(LifterState.STOWED));
  }

  public Command homeAll() {
    Debouncer[] homingDebouncer = new Debouncer[3];
    for (int i = 0; i < 3; i++) homingDebouncer[i] = new Debouncer(0.01);
    Timer homingTimer = new Timer();

    return Commands.startRun(
            () -> {
              io.setVoltage(-LifterConstants.HOMING_VOLTAGE);
            },
            () -> {
              int completed = 0;
              for (int i = 0; i < 3; i++) {
                if (homingDebouncer[i].calculate(
                        Math.abs(inputs.velocityRPS[i])
                            <= LifterConstants.HOMING_VELOCITY_THRESHOLD)
                    && !homingTimer.isRunning()) {
                  io.setVoltage(i, 0);
                  io.resetPosition(i, Rotations.of(0));
                  completed++;
                }
              }
              if (completed == 3) {
                homingTimer.start();
              }
            })
        .until(() -> homingTimer.hasElapsed(0.101))
        .finallyDo(
            () -> {
              homingTimer.stop();
              homingTimer.reset();
            });
  }

  public Command manualControl(DoubleSupplier magnitude) {
    return Commands.startRun(
            () -> {
              for (int i = 0; i < 3; i++) {
                lifterStates[i] = LifterState.MANUAL_CONTROL;
              }
            },
            () -> {
              io.setVoltage(magnitude.getAsDouble() * LifterConstants.MAX_MANUAL_VOLTAGE);
            })
        .andThen(() -> io.setVoltage(0.0));
  }

  public double getLeftPosMeters() {
    return LifterConstants.getPositionMeters(inputs.positionDeg[0]);
  }

  public double getRightPosMeters() {
    return LifterConstants.getPositionMeters(inputs.positionDeg[1]);
  }

  public double getCenterPosMeters() {
    return LifterConstants.getPositionMeters(inputs.positionDeg[2]);
  }
}
