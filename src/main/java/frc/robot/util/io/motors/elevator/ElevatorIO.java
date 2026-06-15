package frc.robot.util.io.motors.elevator;

import edu.wpi.first.units.measure.Angle;
import frc.robot.util.io.motors.MotorIO;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO extends MotorIO {
  @AutoLog
  class ElevatorIOInputs extends MotorIOInputs {
    public double positionDeg;
    public double velocityRPS;
  }

  default void updateInputs(ElevatorIOInputs inputs) {}

  default void setPosition(double deg) {}

  default void resetPosition(Angle angle) {}
}
