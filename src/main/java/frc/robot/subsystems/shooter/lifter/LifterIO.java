package frc.robot.subsystems.shooter.lifter;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface LifterIO {

  @AutoLog
  class LifterIOInputs {
    public double[] positionDeg;
    public double[] velocityRPS;

    public boolean[] connected;
    public double[] appliedVoltage;
    public double[] supplyCurrentAmps;
    public double[] statorCurrentAmps;
    public double[] tempCelsius;
  }

  default void updateInputs(LifterIOInputs inputs) {}

  default void setPosition(int lifter, double deg) {}

  default void setPosition(double deg) {
    setPosition(0, deg);
    setPosition(1, deg);
    setPosition(2, deg);
  }

  default void resetPosition(int lifter, Angle angle) {}

  default void resetPosition(Angle angle) {
    resetPosition(0, angle);
    resetPosition(1, angle);
    resetPosition(2, angle);
  }

  default void setVoltage(int lifter, double volts) {}

  default void setVoltage(double volts) {
    setVoltage(0, volts);
    setVoltage(1, volts);
    setVoltage(2, volts);
  }

  default void stop(int lifter) {}

  default void stop() {
    stop(0);
    stop(1);
    stop(2);
  }

  default void setBrake(int lifter, boolean brake) {}

  default void setBrake(boolean brake) {
    setBrake(0, brake);
    setBrake(1, brake);
    setBrake(2, brake);
  }
}
