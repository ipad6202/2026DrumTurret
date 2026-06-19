// Copyright (c) DrumTurret project contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root of this project.

package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean leaderConnected;
    public Angle leaderAngle;
    public AngularVelocity leaderAngularVelocity;
    public Voltage leaderVoltage;
    public Current leaderCurrent;
    public Temperature leaderTemperature;
  }

  default void updateInputs(HoodIOInputs inputs) {}

  default void setAngle(Angle angle) {}

  default void setVoltage(Voltage volts) {}
}
