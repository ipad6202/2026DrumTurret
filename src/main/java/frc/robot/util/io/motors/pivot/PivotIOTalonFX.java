package frc.robot.util.io.motors.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.io.motors.MotorIOTalonFX;
import frc.robot.util.io.sensors.EncoderIOCANcoder;
import java.util.function.Consumer;

public class PivotIOTalonFX extends MotorIOTalonFX implements PivotIO {
  private Consumer<Angle> positionRequest;

  private final StatusSignal<Angle> position;

  private volatile Angle angleResetVal = Rotations.zero();
  private final Notifier resetPosition = new Notifier(() -> leader.setPosition(angleResetVal));

  public PivotIOTalonFX(CANBus canbus, int id, TalonFXConfiguration config) {
    this(canbus, id, new int[0], config, new MotorAlignmentValue[0]);
  }

  public PivotIOTalonFX(
      CANBus canbus,
      int id,
      int[] followerIds,
      TalonFXConfiguration config,
      MotorAlignmentValue[] followerAlignments) {
    super(canbus, id, followerIds, config, followerAlignments);
    useControlRequest(new PositionVoltage(0).withOverrideBrakeDurNeutral(true));
    position = leader.getPosition();
    position.setUpdateFrequency(100.0);
    PhoenixUtil.registerSignals(canbus, position);
  }

  public PivotIOTalonFX useControlRequest(PositionVoltage request) {
    this.positionRequest = (angle) -> leader.setControl(request.withPosition(angle));
    return this;
  }

  public PivotIOTalonFX useControlRequest(MotionMagicVoltage request) {
    this.positionRequest = (angle) -> leader.setControl(request.withPosition(angle));
    return this;
  }

  public PivotIOTalonFX useControlRequest(MotionMagicExpoVoltage request) {
    this.positionRequest = (angle) -> leader.setControl(request.withPosition(angle));
    return this;
  }

  public PivotIOTalonFX useCANcoder(EncoderIOCANcoder encoder) {
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
  public void updateInputs(PivotIOInputs inputs) {
    inputs.positionDeg = position.getValue().in(Degrees);
    updateMotorInputs(inputs);
  }

  @Override
  public void setPosition(Angle angle) {
    positionRequest.accept(angle);
  }

  @Override
  public void resetPosition(Angle angle) {
    angleResetVal = angle;
    resetPosition.startSingle(0);
  }
}
