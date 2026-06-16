package frc.robot.subsystems.shooter.lifter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class LifterIOSim implements LifterIO {
  public final ElevatorSim[] lifterSims = new ElevatorSim[3];

  private final double gearing;
  private final double drumRadiusMeters;

  private double[] appliedVoltage = new double[3];
  private final PIDController pid;
  private boolean[] isClosedLoop = new boolean[3];

  private double[] targetPositionDegs = new double[3];

  public LifterIOSim() {
    this.gearing = LifterConstants.ELEVATOR_GEAR_RATIO;
    this.drumRadiusMeters = LifterConstants.DRUM_RADIUS;

    this.lifterSims[0] =
        new ElevatorSim(
            DCMotor.getKrakenX44(1),
            this.gearing,
            LifterConstants.CARRIAGE_MASS,
            this.drumRadiusMeters,
            LifterConstants.MIN_HEIGHT,
            LifterConstants.MAX_HEIGHT,
            true,
            0.0);
    this.lifterSims[1] =
        new ElevatorSim(
            DCMotor.getKrakenX44(1),
            this.gearing,
            LifterConstants.CARRIAGE_MASS,
            this.drumRadiusMeters,
            LifterConstants.MIN_HEIGHT,
            LifterConstants.MAX_HEIGHT,
            true,
            0.0);
    this.lifterSims[2] =
        new ElevatorSim(
            DCMotor.getKrakenX44(1),
            this.gearing,
            LifterConstants.CARRIAGE_MASS,
            this.drumRadiusMeters,
            LifterConstants.MIN_HEIGHT,
            LifterConstants.MAX_HEIGHT,
            true,
            0.0);

    pid = new PIDController(LifterConstants.ELEVATOR_KP, 0.0, LifterConstants.ELEVATOR_KD);
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

  public void updateInputs(LifterIOInputs inputs) {
    for (int i = 0; i < 3; i++) {
      if (isClosedLoop[i]) {
        double currentMotorDeg = carriageMetersToDeg(lifterSims[i].getPositionMeters());
        appliedVoltage[i] =
            MathUtil.clamp(pid.calculate(currentMotorDeg, targetPositionDegs[i]), -12, 12);
      }

      lifterSims[i].setInputVoltage(appliedVoltage[i]);
      lifterSims[i].update(0.020);
      inputs.positionDeg[i] = carriageMetersToDeg(lifterSims[i].getPositionMeters());
      inputs.velocityRPS[i] = carriageMPSToRPS(lifterSims[i].getVelocityMetersPerSecond());
      inputs.statorCurrentAmps[i] = lifterSims[i].getCurrentDrawAmps();
      inputs.supplyCurrentAmps[i] = appliedVoltage[i] / 12.0 * inputs.statorCurrentAmps[i];

      inputs.connected[i] = true;
      inputs.appliedVoltage[i] = appliedVoltage[i];
      inputs.tempCelsius[i] = 0.0;
    }
  }

  public void setPosition(int lifter, double deg) {
    this.targetPositionDegs[lifter] = deg;
    isClosedLoop[lifter] = true;
  }

  public void resetPosition(int lifter, Angle angle) {
    this.targetPositionDegs[lifter] = angle.in(Degrees);
    lifterSims[lifter].setState(rotationsToCarriageMeters(angle.in(Rotations)), 0.0);
  }

  public void setVoltage(int lifter, double volts) {
    isClosedLoop[lifter] = false;
    appliedVoltage[lifter] = volts;
  }

  public void stop(int lifter) {
    isClosedLoop[lifter] = false;
    appliedVoltage[lifter] = 0.0;
  }

  public void setBrake(boolean brake) {}
}
