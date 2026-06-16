package frc.robot.subsystems.shooter.lifter;

import java.util.EnumMap;

public class LifterConstants {
  public enum LifterState {
    STOWED,
    RAISED,
    LOWERING,
    HOMING,
    MANUAL_CONTROL
  }

  public static double getPositionMeters(double positionDeg) {
    double rotations = positionDeg / 360.0;
    return rotations
        * 2.0
        * Math.PI
        * LifterConstants.DRUM_RADIUS
        / LifterConstants.ELEVATOR_GEAR_RATIO;
  }

  public static double metersToDeg(double meters) {
    return meters / DRUM_RADIUS * 180 / Math.PI * ELEVATOR_GEAR_RATIO;
  }

  public static final double ELEVATOR_GEAR_RATIO = 75;
  public static final double CARRIAGE_MASS = 15;
  public static final double DRUM_RADIUS = 0.2;
  public static final double MAX_HEIGHT = 1;
  public static final double MIN_HEIGHT = 0;
  public static final double ELEVATOR_KP = 0.1;
  public static final double ELEVATOR_KD = 0;

  public static final double MAX_MANUAL_VOLTAGE = 6.0;
  public static final double HOMING_VOLTAGE = 2.0;
  public static final double HOMING_VELOCITY_THRESHOLD = 1.0;

  public static final EnumMap<LifterState, Double> SETPOINTS = new EnumMap<>(LifterState.class);

  static { // placeholders
    SETPOINTS.put(LifterState.STOWED, metersToDeg(0));
    SETPOINTS.put(LifterState.RAISED, metersToDeg(1));
    SETPOINTS.put(LifterState.LOWERING, metersToDeg(0.1));
  }
}
