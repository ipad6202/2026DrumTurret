// Copyright (c) DrumTurret project contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root of this project.

package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim implements TurretIO {
  // TODO: Figure out
  private final double kV = 1.0;
  private final double kA = 0.01;

  private final DCMotorSim turretSim =
      new DCMotorSim(LinearSystemId.createDCMotorSystem(kV, kA), DCMotor.getKrakenX60(1));

  private final PIDController controller = new PIDController(1.0, 0.0, 0.0);

  public TurretIOSim() {
    controller.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void run(double thetaRad, double omegaRadPerSec) {
    turretSim.setInputVoltage(
        controller.calculate(turretSim.getAngularVelocityRadPerSec(), omegaRadPerSec)
            + omegaRadPerSec * kV);
  }
}
