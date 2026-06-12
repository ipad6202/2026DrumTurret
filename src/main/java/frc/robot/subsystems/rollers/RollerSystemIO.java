package frc.robot.subsystems.rollers;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface RollerSystemIO {

  @AutoLog
  public static class RollerSystemIOInputs {
    public boolean connected;
    public Angle leaderAngle;
    public AngularVelocity leaderAngularVelocity;
    public Voltage leaderVoltage;
    public Current leaderCurrent;
    public Temperature leaderTemperature;

    public boolean hasFollower;
    public boolean followerConnected;
    public Angle followerAngle;
    public AngularVelocity followerAngularVelocity;
    public Voltage followerVoltage;
    public Current followerCurrent;
    public Temperature followerTemperature;
  }

  default void updateInputs(RollerSystemIOInputs inputs) {}

  default void setVoltage(Voltage volts) {}

  default void setVelocity(AngularVelocity velocity) {}

  default void stop() {}

  default void setBrakeMode(boolean enabled) {}
}
