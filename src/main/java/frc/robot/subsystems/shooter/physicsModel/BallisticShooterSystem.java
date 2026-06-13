package frc.robot.subsystems.shooter.physicsModel;

public final class BallisticShooterSystem {

  public record FullResult(
      ShooterPhysics.ShotResult shot, BallisticCalculator.TrajectoryResult trajectory) {}

  public static FullResult simulateFullShot(
      double mb,
      double rb,
      double Ib,
      double rw,
      double Iw,
      double wheelSpeed,
      double launchAngleRad,
      double shooterHeight,
      double Cd) {

    // 1. Shooter exit model
    var shot =
        ShooterPhysics.calculateShot(
            mb,
            rb,
            Ib,
            rw,
            Iw,
            0.0, // Vbi
            0.0, // omegaBi
            wheelSpeed);

    // 2. Projectile model
    var traj =
        BallisticCalculator.simulate(
            shot.exitVelocity(), launchAngleRad, shooterHeight, mb, rb, Cd);

    return new FullResult(shot, traj);
  }
}
