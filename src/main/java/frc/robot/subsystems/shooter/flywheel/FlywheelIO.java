package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

  @AutoLog
  public static class FlywheelIOInputs {
    public boolean leaderConnected;
    public Angle leaderAngle;
    public AngularVelocity leaderAngularVelocity;
    public Voltage leaderVoltage;
    public Current leaderCurrent;
    public Temperature leaderTemperature;

    public boolean follower1Connected;
    public Angle follower1Angle;
    public AngularVelocity follower1AngularVelocity;
    public Voltage follower1Voltage;
    public Current follower1Current;
    public Temperature follower1Temperature;

    public boolean follower2Connected;
    public Angle follower2Angle;
    public AngularVelocity follower2AngularVelocity;
    public Voltage follower2Voltage;
    public Current follower2Current;
    public Temperature follower2Temperature;
  }

  default void updateInputs(FlywheelIOInputs inputs) {}

  default void setVelocity(AngularVelocity velocity) {}

  default void stop() {}
}
