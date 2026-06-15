package frc.robot.util.io.motors.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.util.io.motors.MotorIOSim;

public class ElevatorIOSim extends MotorIOSim implements ElevatorIO {
  private final ElevatorSim sim;

  private final double gearing;
  private final double drumRadiusMeters;

  private double targetPositionDeg = 0.0;

  public ElevatorIOSim(
      DCMotor gearbox,
      double gearing,
      double carriageMassKg,
      double drumRadiusMeters,
      double minHeightMeters,
      double maxHeightMeters,
      double kP,
      double kD,
      int numFollowers) {
    super(kP, kD, numFollowers);

    this.gearing = gearing;
    this.drumRadiusMeters = drumRadiusMeters;

    this.sim =
        new ElevatorSim(
            gearbox,
            gearing,
            carriageMassKg,
            drumRadiusMeters,
            minHeightMeters,
            maxHeightMeters,
            true,
            0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (isClosedLoop) {
      double currentMotorDeg = carriageMetersToDeg(sim.getPositionMeters());
      appliedVoltage = MathUtil.clamp(pid.calculate(currentMotorDeg, targetPositionDeg), -12, 12);
    }
    updateMotorInputs(inputs);

    sim.setInputVoltage(appliedVoltage);
    sim.update(0.020);
    inputs.positionDeg = carriageMetersToDeg(sim.getPositionMeters());
    inputs.velocityRPS = carriageMPSToRPS(sim.getVelocityMetersPerSecond());
    inputs.statorCurrentAmps = sim.getCurrentDrawAmps();
    inputs.supplyCurrentAmps = appliedVoltage / 12.0 * inputs.statorCurrentAmps;
  }

  @Override
  public void setPosition(double deg) {
    this.targetPositionDeg = deg;
    isClosedLoop = true;
  }

  @Override
  public void resetPosition(Angle angle) {
    this.targetPositionDeg = angle.in(Degrees);
    sim.setState(rotationsToCarriageMeters(angle.in(Rotations)), 0.0);
  }

  private double carriageMetersToDeg(double meters) {
    double drumRotations = meters / (2.0 * Math.PI * drumRadiusMeters);
    double motorRotations = drumRotations * gearing;
    return motorRotations * 360.0;
  }

  private double rotationsToCarriageMeters(double rotations) {
    double drumRotations = rotations / gearing;
    return drumRotations * (2.0 * Math.PI * drumRadiusMeters);
  }

  private double carriageMPSToRPS(double metersPerSecond) {
    double drumRPS = metersPerSecond / (2.0 * Math.PI * drumRadiusMeters);
    return drumRPS * gearing;
  }
}
