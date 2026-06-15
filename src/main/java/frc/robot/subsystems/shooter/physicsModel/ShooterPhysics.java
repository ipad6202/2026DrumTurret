// Copyright (c) DrumTurret project contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root of this project.

package frc.robot.subsystems.shooter.physicsModel;

public final class ShooterPhysics {
  private ShooterPhysics() {}

  public record ShotResult(double exitVelocity, double exitSpin, double finalWheelSpeed) {}

  /**
   * SI UNITS ONLY: mass (kg), radius (m), inertia (kg*m^2), velocity (m/s), angular velocity
   * (rad/s)
   */
  public static ShotResult calculateShot(
      double mb,
      double rb,
      double Ib,
      double rw,
      double Iw,
      double Vbi,
      double omegaBi,
      double omegaWi) {
    double D = 4.0 * Iw * rb * rb + Ib * rw * rw + mb * rw * rw * rb * rb;

    double Vbf =
        (2.0 * Iw * rw * rb * rb * omegaWi
                + Ib * rw * rw * rb * omegaBi
                + mb * rw * rw * rb * rb * Vbi)
            / D;

    double omegaBf =
        (2.0 * Iw * rw * rb * omegaWi + Ib * rw * rw * omegaBi + mb * rw * rw * rb * Vbi) / D;

    double omegaWf =
        (4.0 * Iw * rb * rb * omegaWi
                + 2.0 * Ib * rw * rb * omegaBi
                + 2.0 * mb * rw * rb * rb * Vbi)
            / D;

    return new ShotResult(Vbf, omegaBf, omegaWf);
  }
}
