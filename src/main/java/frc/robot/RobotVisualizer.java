package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class RobotVisualizer {
  private final DoubleSupplier turretAngleSupplier;
  private final DoubleSupplier hoodAngleSupplier;
  private final DoubleSupplier intakeAngleSupplier;
  private final DoubleSupplier floorAngleSupplier;
  private final DoubleSupplier frontLiftsHeightSupplier;
  private final DoubleSupplier backLiftHeightSupplier;

  public RobotVisualizer(
      DoubleSupplier turretYawSupplier,
      DoubleSupplier hoodAngleSupplier,
      DoubleSupplier intakeAngleSupplier,
      DoubleSupplier floorAngleSupplier,
      DoubleSupplier frontLiftsHeightSupplier,
      DoubleSupplier backLiftHeightSupplier) {
    this.turretAngleSupplier = turretYawSupplier;
    this.hoodAngleSupplier = hoodAngleSupplier;
    this.intakeAngleSupplier = intakeAngleSupplier;
    this.floorAngleSupplier = floorAngleSupplier;
    this.frontLiftsHeightSupplier = frontLiftsHeightSupplier;
    this.backLiftHeightSupplier = backLiftHeightSupplier;
  }

  public static final Translation3d TURRET_PIVOT_ZERO = new Translation3d(0.438150, 0, 0.694512);
  public static final Translation3d HOOD_ZERO = new Translation3d(-0.118910, 0, 0.694512);
  public static final Translation3d INTAKE_ZERO = new Translation3d(-0.057320, 0, 0.308996);
  public static final Translation3d EXPANDING_FLOOR_PIVOT_ZERO =
      new Translation3d(0.080264, 0, 0.204390);
  public static final double MAX_LIFT_HEIGHT = Units.inchesToMeters(17.75);
  public static final double MAX_HORIZONTAL_HOPPER_EXTENSION = Units.inchesToMeters(12);
  public static final double MAX_VERTICAL_HOPPER_EXTENSION = Units.inchesToMeters(9);
  public static final double LAUNCHER_FRONT_PIVOT_POINT = -0.186314098902; // meters
  public static final double LAUNCHER_PIVOT_LENGTH =
      TURRET_PIVOT_ZERO.getX() - LAUNCHER_FRONT_PIVOT_POINT;

  public void periodic() {
    double turretYaw = turretAngleSupplier.getAsDouble();
    double hoodAngle = hoodAngleSupplier.getAsDouble();
    double intakeAngle = intakeAngleSupplier.getAsDouble();
    double floorAngle = floorAngleSupplier.getAsDouble();
    double frontVertExt = frontLiftsHeightSupplier.getAsDouble();
    double backVertExt = backLiftHeightSupplier.getAsDouble();

    Logger.recordOutput("RobotVisualizer/turretYaw", turretYaw);
    Logger.recordOutput("RobotVisualizer/hoodAngle", hoodAngle);
    Logger.recordOutput("RobotVisualizer/intakeAngle", intakeAngle);
    Logger.recordOutput("RobotVisualizer/floorAngle", floorAngle);
    Logger.recordOutput("RobotVisualizer/frontVertExt", frontVertExt);
    Logger.recordOutput("RobotVisualizer/backVertExt", backVertExt);

    double frontZOffset = -(MAX_LIFT_HEIGHT - frontVertExt);
    double backZOffset = -(MAX_LIFT_HEIGHT - backVertExt);
    double frontLowerZOffset = -(MAX_LIFT_HEIGHT - Math.max(frontVertExt, MAX_LIFT_HEIGHT / 2.));
    double backLowerZOffset = -(MAX_LIFT_HEIGHT - Math.max(backVertExt, MAX_LIFT_HEIGHT / 2.));

    double horizontalExtension =
        MAX_HORIZONTAL_HOPPER_EXTENSION
            - Math.max(0, MAX_HORIZONTAL_HOPPER_EXTENSION * Math.cos(intakeAngle));
    double verticalExtension =
        -(MAX_LIFT_HEIGHT - Math.max(frontVertExt, MAX_VERTICAL_HOPPER_EXTENSION));

    double launcherAngle = -(backVertExt - frontVertExt) / LAUNCHER_PIVOT_LENGTH;

    Transform3d turretBase =
        new Transform3d(
            TURRET_PIVOT_ZERO.plus(new Translation3d(0, 0, backZOffset)),
            new Rotation3d(0, launcherAngle, 0));
    Transform3d turretRotary =
        turretBase.plus(
            new Transform3d(TURRET_PIVOT_ZERO.unaryMinus(), new Rotation3d(0, 0, -turretYaw)));
    Transform3d hood =
        turretRotary.plus(new Transform3d(HOOD_ZERO, new Rotation3d(0, -hoodAngle, 0)));

    Transform3d intake = new Transform3d(INTAKE_ZERO, new Rotation3d(0, intakeAngle, 0));

    Transform3d floorPivot =
        new Transform3d(EXPANDING_FLOOR_PIVOT_ZERO, new Rotation3d(0, floorAngle, 0));

    Transform3d horizontalHopper =
        new Transform3d(new Translation3d(horizontalExtension, 0, 0), Rotation3d.kZero);
    Transform3d verticalHopper =
        new Transform3d(new Translation3d(0, 0, verticalExtension), Rotation3d.kZero);
    Transform3d diagonalHopper =
        new Transform3d(
            new Translation3d(horizontalExtension, 0, verticalExtension), Rotation3d.kZero);

    Transform3d frontUpper =
        new Transform3d(new Translation3d(0, 0, frontZOffset), Rotation3d.kZero);
    Transform3d frontLower =
        new Transform3d(new Translation3d(0, 0, frontLowerZOffset), Rotation3d.kZero);

    Transform3d backUpper = new Transform3d(new Translation3d(0, 0, backZOffset), Rotation3d.kZero);
    Transform3d backLower =
        new Transform3d(new Translation3d(0, 0, backLowerZOffset), Rotation3d.kZero);

    Logger.recordOutput(
        "RobotVisualizer/Components",
        new Transform3d[] {
          turretBase,
          turretRotary,
          hood,
          intake,
          floorPivot,
          horizontalHopper,
          verticalHopper,
          diagonalHopper,
          frontUpper,
          frontLower,
          backUpper,
          backLower
        });

    Logger.recordOutput("RobotVisualizer/Origin", Pose3d.kZero); // for testing
  }
}
