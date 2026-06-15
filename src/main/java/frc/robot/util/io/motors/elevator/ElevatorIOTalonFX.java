package frc.robot.util.io.motors.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.io.motors.MotorIOTalonFX;
import frc.robot.util.io.sensors.EncoderIOCANcoder;
import java.util.function.DoubleConsumer;

public class ElevatorIOTalonFX extends MotorIOTalonFX implements ElevatorIO {
  private DoubleConsumer positionRequest;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;

  private volatile Angle angleResetVal = Rotations.zero();
  private final Notifier resetPosition = new Notifier(() -> leader.setPosition(angleResetVal));

  public ElevatorIOTalonFX(CANBus canbus, int id, TalonFXConfiguration config) {
    this(canbus, id, new int[0], config, new MotorAlignmentValue[0]);
  }

  public ElevatorIOTalonFX(
      CANBus canbus,
      int id,
      int[] followerIds,
      TalonFXConfiguration config,
      MotorAlignmentValue[] followerAlignments) {
    super(canbus, id, followerIds, config, followerAlignments);
    useControlRequest(new PositionVoltage(0).withOverrideBrakeDurNeutral(true));
    position = leader.getPosition();
    velocity = leader.getVelocity();
    position.setUpdateFrequency(100.0);
    velocity.setUpdateFrequency(100.0);
    PhoenixUtil.registerSignals(canbus, position, velocity);
  }

  public ElevatorIOTalonFX useControlRequest(PositionVoltage request) {
    this.positionRequest =
        (deg) -> leader.setControl(request.withPosition(Units.degreesToRotations(deg)));
    return this;
  }

  public ElevatorIOTalonFX useControlRequest(MotionMagicVoltage request) {
    this.positionRequest =
        (deg) -> leader.setControl(request.withPosition(Units.degreesToRotations(deg)));
    return this;
  }

  public ElevatorIOTalonFX useControlRequest(MotionMagicExpoVoltage request) {
    this.positionRequest =
        (deg) -> leader.setControl(request.withPosition(Units.degreesToRotations(deg)));
    return this;
  }

  public ElevatorIOTalonFX useCANcoder(EncoderIOCANcoder encoder) {
    tryUntilOk(
        5,
        () ->
            leader
                .getConfigurator()
                .apply(
                    new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                        .withFeedbackRemoteSensorID(encoder.getDeviceID())));
    return this;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.positionDeg = position.getValue().in(Degrees);
    inputs.velocityRPS = velocity.getValue().in(RotationsPerSecond);
    updateMotorInputs(inputs);
  }

  @Override
  public void setPosition(double deg) {
    positionRequest.accept(deg);
  }

  @Override
  public void resetPosition(Angle angle) {
    angleResetVal = angle;
    resetPosition.startSingle(0);
  }
}
