// Copyright (c) DrumTurret project contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.turret.TurretIO;
import java.util.function.Supplier;

public class Shooter extends SubsystemBase {
  private static final ShotMap hubShotMap = ShotMap.loadFromDeploy("HubShotMap.json");
  private static final ShotMap groundShotMap = ShotMap.loadFromDeploy("GroundShotMap.json");

  // Location that the robot should shoot at for passing balls
  private static final Translation2d BLUE_RIGHT_GROUND_TARGET =
      new Translation2d(
          FieldConstants.LinesVertical.starting - 1, FieldConstants.Hub.nearRightCorner.getY() / 2);
  private static final Translation2d BLUE_LEFT_GROUND_TARGET =
      new Translation2d(
          BLUE_RIGHT_GROUND_TARGET.getX(),
          FieldConstants.fieldWidth - BLUE_RIGHT_GROUND_TARGET.getY());
  private static final Translation2d RED_LEFT_GROUND_TARGET =
      new Translation2d(
          FieldConstants.fieldLength - BLUE_RIGHT_GROUND_TARGET.getX(),
          BLUE_RIGHT_GROUND_TARGET.getY());
  private static final Translation2d RED_RIGHT_GROUND_TARGET =
      new Translation2d(
          RED_LEFT_GROUND_TARGET.getX(), FieldConstants.fieldWidth - RED_LEFT_GROUND_TARGET.getY());

  public enum ShotTarget {
    BLUE_HUB(hubShotMap, FieldConstants.Hub.topCenterPoint.toTranslation2d()),
    BLUE_RIGHT_GROUND(groundShotMap, BLUE_RIGHT_GROUND_TARGET),
    BLUE_LEFT_GROUND(groundShotMap, BLUE_LEFT_GROUND_TARGET),
    RED_HUB(hubShotMap, FieldConstants.Hub.oppTopCenterPoint.toTranslation2d()),
    RED_LEFT_GROUND(groundShotMap, RED_LEFT_GROUND_TARGET),
    RED_RIGHT_GROUND(groundShotMap, RED_RIGHT_GROUND_TARGET);

    public final ShotMap map;
    public final Translation2d targetLocation;

    ShotTarget(ShotMap map, Translation2d targetLocation) {
      this.map = map;
      this.targetLocation = targetLocation;
    }
  }

  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> drivetrainSpeedsSupplier;

  private final TurretIO turretIO;

  private final InterpolatingDoubleTreeMap shotSpeedToFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();

  private final double linearDragCoefficientInverseSeconds = 0.1;

  public Shooter(
      TurretIO turretIO,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> drivetrainSpeedsSupplier) {
    this.turretIO = turretIO;

    this.robotPoseSupplier = robotPoseSupplier;
    this.drivetrainSpeedsSupplier = drivetrainSpeedsSupplier;

    // TODO: Figure out shot speed -> flywheel speed mapping
    shotSpeedToFlywheelSpeedMap.put(0.0, 0.0);
  }

  public Command shootHub() {
    return run(
        () ->
            shoot(
                robotPoseSupplier.get(),
                Robot.isOnRed()
                    ? ShotTarget.RED_HUB.targetLocation
                    : ShotTarget.BLUE_HUB.targetLocation,
                hubShotMap));
  }

  public Command pass() {
    return run(
        () -> {
          var robotPose = robotPoseSupplier.get();
          if (robotPose.getY() < FieldConstants.fieldWidth / 2) {
            shoot(
                robotPoseSupplier.get(),
                Robot.isOnRed()
                    ? ShotTarget.RED_LEFT_GROUND.targetLocation
                    : ShotTarget.BLUE_RIGHT_GROUND.targetLocation,
                hubShotMap);
          } else {
            shoot(
                robotPoseSupplier.get(),
                Robot.isOnRed()
                    ? ShotTarget.RED_RIGHT_GROUND.targetLocation
                    : ShotTarget.BLUE_LEFT_GROUND.targetLocation,
                groundShotMap);
          }
        });
  }

  private void shoot(Pose2d robotPose, Translation2d target, ShotMap shotMap) {
    var drivetrainSpeeds = drivetrainSpeedsSupplier.get();

    var height = 0.0; // TODO: Get height from lifter subsystem

    var timeOfFlight = shotMap.getTimeOfFlight(height);

    var virtualRobotTranslation =
        robotPose
            .getTranslation()
            .plus(
                new Translation2d(
                    drivetrainSpeeds.vxMetersPerSecond * timeOfFlight,
                    drivetrainSpeeds.vyMetersPerSecond * timeOfFlight));

    var virtualRobotToTarget = target.minus(virtualRobotTranslation);
    var aimAngleAbsolute = virtualRobotToTarget.getAngle();
    var aimAngleRelative = aimAngleAbsolute.minus(robotPose.getRotation());
    var shotDistance = virtualRobotToTarget.getNorm();

    var shotParameters = shotMap.get(height, shotDistance);

    // TODO: Actually run the shooter system
    var flywheelSpeedRotPerSec =
        shotSpeedToFlywheelSpeedMap.get(shotParameters.speedMetersPerSec());
    var hoodPitchRad = shotParameters.pitchRad();

    // TODO: Add unwrapping logic if needed
    turretIO.run(aimAngleRelative.getRadians(), -drivetrainSpeeds.omegaRadiansPerSecond);
  }
}
