// Copyright (c) DrumTurret project contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root of this project.

package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  @AutoLogOutput private AngularVelocity targetVelocity = RadiansPerSecond.zero();

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Flywheel", inputs);
  }

  private void setVelocity(AngularVelocity velocity) {
    targetVelocity = velocity;
    io.setVelocity(velocity);
  }

  private void stop() {
    targetVelocity = RadiansPerSecond.zero();
    io.stop();
  }

  private Command runVelocityCommand(Supplier<AngularVelocity> velocitySupplier) {
    return runEnd(() -> setVelocity(velocitySupplier.get()), this::stop);
  }

  @AutoLogOutput(key = "Shooter/Flywheel/isAtGoalVelocity")
  public boolean isAtGoalVelocity() {
    return inputs.leaderAngularVelocity.isNear(
        targetVelocity, FlywheelConstants.VELOCITY_TOLERANCE);
  }
}
