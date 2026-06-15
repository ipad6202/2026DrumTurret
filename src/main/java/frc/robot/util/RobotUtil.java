package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import lombok.Setter;

/**
 * This class contains methods that are used throughout the codebase and are not bound to one
 * subsystem or class.
 */
public final class RobotUtil {
  public static boolean isPoseEstimatorReady;
  @Setter private static CommandXboxController driverController;
  @Setter private static CommandXboxController operatorController;

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  public static boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }

  /**
   * Set the rumble output for the driver controller without needing its instance.
   *
   * @param leftRumble The normalized value (0 to 1) to set the left rumble to
   * @param rightRumble The normalized value (0 to 1) to set the right rumble to
   */
  public static void setDriverRumble(double leftRumble, double rightRumble) {
    driverController.setRumble(GenericHID.RumbleType.kLeftRumble, leftRumble);
    driverController.setRumble(GenericHID.RumbleType.kRightRumble, rightRumble);
  }

  /**
   * Set the rumble output for the operator controller without needing its instance.
   *
   * @param leftRumble The normalized value (0 to 1) to set the left rumble to
   * @param rightRumble The normalized value (0 to 1) to set the right rumble to
   */
  public static void setOperatorRumble(double leftRumble, double rightRumble) {
    operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, leftRumble);
    operatorController.setRumble(GenericHID.RumbleType.kRightRumble, rightRumble);
  }
}
