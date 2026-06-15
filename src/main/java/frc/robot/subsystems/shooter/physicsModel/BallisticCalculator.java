// Copyright (c) DrumTurret project contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root of this project.

package frc.robot.subsystems.shooter.physicsModel;

import java.util.ArrayList;
import java.util.List;

public final class BallisticCalculator {
  private static final double GRAVITY = 9.81;
  private static final double AIR_DENSITY = 1.225;

  public record TrajectoryResult(
      double range, double flightTime, double impactY, List<Point> samples) {}

  public record Point(double x, double y, double vx, double vy) {}

  public static TrajectoryResult simulate(
      double initialSpeed,
      double launchAngleRad,
      double height,
      double mass,
      double radius,
      double Cd) {
    double dt = 0.001;

    double area = Math.PI * radius * radius;

    double x = 0.0;
    double y = height;

    double vx = initialSpeed * Math.cos(launchAngleRad);
    double vy = initialSpeed * Math.sin(launchAngleRad);

    double time = 0.0;

    List<Point> samples = new ArrayList<>();

    int iterations = 0;
    while (y > 0.0 && time < 10.0) {
      double v = Math.hypot(vx, vy);

      double ax = 0.0;
      double ay = -GRAVITY;

      if (v > 1e-6) {
        double drag = 0.5 * AIR_DENSITY * Cd * area * v * v;
        double aDrag = drag / mass;

        ax -= aDrag * (vx / v);
        ay -= aDrag * (vy / v);
      }

      vx += ax * dt;
      vy += ay * dt;

      x += vx * dt;
      y += vy * dt;

      time += dt;

      iterations++;
      if (iterations % 10 == 0) samples.add(new Point(x, y, vx, vy));
    }

    return new TrajectoryResult(x, time, y, samples);
  }
}
