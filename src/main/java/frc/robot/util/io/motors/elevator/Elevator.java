package frc.robot.util.io.motors.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.io.motors.MotorIO;
import frc.robot.util.subsystems.RobotStateHandler;
import java.util.EnumMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator<T extends Enum<T>>{
  private final String name;
  private final T[] elevatorStates;
  private final EnumMap<T, Double> setpoints;

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private T setpoint;
  private double setpointDeg;

  private final BooleanSupplier brakeDurNeutral;
  private final Alert torqueLimitWarning;
  private final Alert tempWarning;
  private final Alert tempFault;
  private boolean stalled;
  private boolean tempCritical;

  private final double HOMING_VOLTAGE;
  private final double HOMING_VELOCITY_THRESHOLD;
  private final double MAX_MANUAL_VOLTAGE;

  private MotorIO.MotorIOMode mode;

  public Elevator(
      String name,
      ElevatorIO io,
      Class<T> elevatorStateClass,
      EnumMap<T, Double> setpoints,
      double HOMING_VOLTAGE,
      double HOMING_VELOCITY_THRESHOLD,
      double MAX_MANUAL_VOLTAGE) {
    this(
        name,
        io,
        RobotStateHandler::isEnabled,
        40.0,
        elevatorStateClass,
        setpoints,
        HOMING_VOLTAGE,
        HOMING_VELOCITY_THRESHOLD,
        MAX_MANUAL_VOLTAGE);
  }

  public Elevator(
      String name,
      ElevatorIO io,
      double currentLimit,
      Class<T> elevatorStateClass,
      EnumMap<T, Double> setpoints,
      double HOMING_VOLTAGE,
      double HOMING_VELOCITY_THRESHOLD,
      double MAX_MANUAL_VOLTAGE) {
    this(
        name,
        io,
        RobotStateHandler::isEnabled,
        currentLimit,
        elevatorStateClass,
        setpoints,
        HOMING_VOLTAGE,
        HOMING_VELOCITY_THRESHOLD,
        MAX_MANUAL_VOLTAGE);
  }

  public Elevator(
      String name,
      ElevatorIO io,
      BooleanSupplier brakeMode,
      double currentLimit,
      Class<T> elevatorStateClass,
      EnumMap<T, Double> setpoints,
      double HOMING_VOLTAGE,
      double HOMING_VELOCITY_THRESHOLD,
      double MAX_MANUAL_VOLTAGE) {
    this.name = name;
    this.io = io;
    this.elevatorStates = elevatorStateClass.getEnumConstants();
    this.setpoints = setpoints;
    this.brakeDurNeutral = brakeMode;
    this.HOMING_VOLTAGE = HOMING_VOLTAGE;
    this.HOMING_VELOCITY_THRESHOLD = HOMING_VELOCITY_THRESHOLD;
    this.MAX_MANUAL_VOLTAGE = MAX_MANUAL_VOLTAGE;

    // Initialize telemetry arrays from IO layer
    inputs.followerConnected = new boolean[io.getNumFollowers()];
    inputs.followerTempCelsius = new double[io.getNumFollowers()];

    // Hardware Alert tracking setup
    torqueLimitWarning =
        new Alert(
            name, "Motor torque limited, disabling to prevent damage", Alert.AlertType.kWarning);
    tempWarning = new Alert(name, "Motor temperature above 60°C", Alert.AlertType.kWarning);
    tempFault =
        new Alert(name, "Motor disabled due to temperature above 75°C", Alert.AlertType.kError);

    // Current monitoring safety trigger
    new Trigger(() -> inputs.statorCurrentAmps >= currentLimit)
        .debounce(0.3, Debouncer.DebounceType.kBoth)
        .onTrue(
            Commands.runOnce(
                () -> {
                  stalled = true;
                  stop(); // Safely cut power / drop to coast
                  torqueLimitWarning.set(true);
                }))
        .onFalse(
            Commands.runOnce(
                () -> {
                  stalled = false;
                  torqueLimitWarning.set(false);
                }));
  }

  public T toElevatorState(int level) {
    return elevatorStates[level];
  }

  private T getStateByName(String stateName) {
    for (T state : elevatorStates) {
      if (state.name().equalsIgnoreCase(stateName)) {
        return state;
      }
    }
    return null;
  }

  public void disable() {
    if (DriverStation.isEStopped()) {
      io.setVoltage(0);
    } else {
      stop();
    }
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);

    // Thermal monitoring matching Pivot
    double highestTemp = inputs.tempCelsius;
    for (double temp : inputs.followerTempCelsius) {
      highestTemp = Math.max(highestTemp, temp);
    }

    if (highestTemp > 75.0) {
      tempCritical = true;
      stop();
      tempFault.set(true);
    } else {
      tempCritical = false;
      tempFault.set(false);
      tempWarning.set(highestTemp > 60.0);
    }
  }

  public void setState(T newState) {
    if (tempCritical || stalled || newState == null) {
      stop();
      return;
    }

    String stateName = newState.name();
    if (stateName.equals("HOMING") || stateName.equals("MANUAL_CONTROL")) {
      // skip
    } else {
      Double newSetpoint = setpoints.get(newState);
      if (newSetpoint != null) {
        io.setPosition(newSetpoint);
        setpointDeg = newSetpoint;
        mode = MotorIO.MotorIOMode.POSITION_CONTROL;
        Logger.recordOutput(name + "/MotorMode", mode);
      }
    }

    setpoint = newState;
    Logger.recordOutput(name + "/ElevatorState", setpoint);
  }

  public void stop() {
    if (brakeDurNeutral.getAsBoolean()) {
      io.brake();
      mode = MotorIO.MotorIOMode.BRAKE;
    } else {
      io.coast();
      mode = MotorIO.MotorIOMode.COAST;
    }
    Logger.recordOutput(name + "/SetpointDeg", 0.0);
    Logger.recordOutput(name + "/MotorMode", mode);
  }

  public double getPositionDeg() {
    return inputs.positionDeg;
  }

  public boolean hasReachedSetpoint() {
    if (setpoint == null) return false;
    return Math.abs(setpointDeg - inputs.positionDeg) < 10;
  }

  public Command stow() {
    T stowedState = getStateByName("STOWED");
    return Commands.startEnd(
            () -> {
              if (stowedState != null) setState(stowedState);
            },
            () -> stop())
        .until(this::hasReachedSetpoint);
  }

  public Command homingSequence() {
    Debouncer homingDebouncer = new Debouncer(0.1);
    Timer homingTimer = new Timer();
    T homingState = getStateByName("HOMING");
    T stowedState = getStateByName("STOWED");

    return Commands.startRun(
            () -> {
              if (homingState != null) setState(homingState);
              io.setVoltage(-HOMING_VOLTAGE);
            },
            () -> {
              if (homingDebouncer.calculate(
                      Math.abs(inputs.velocityRPS) <= HOMING_VELOCITY_THRESHOLD)
                  && !homingTimer.isRunning()) {
                io.setVoltage(0);
                io.resetPosition(Rotations.of(0));
                homingTimer.start();
              }
            })
        .until(() -> homingTimer.hasElapsed(0.101))
        .finallyDo(
            () -> {
              homingTimer.stop();
              homingTimer.reset();
              if (stowedState != null) setState(stowedState);
            });
  }

  public Command manualControl(DoubleSupplier magnitude) {
    T manualState = getStateByName("MANUAL_CONTROL");
    return Commands.startRun(
        () -> {
          if (manualState != null) {
            setState(manualState);
            mode = MotorIO.MotorIOMode.VOLTAGE_CONTROL;
            Logger.recordOutput(name + "/MotorMode", mode);
          }
        },
        () -> {
          if (!stalled && !tempCritical) {
            io.setVoltage(magnitude.getAsDouble() * MAX_MANUAL_VOLTAGE);
          }
        }).andThen(() -> io.setVoltage(0));
  }
}
