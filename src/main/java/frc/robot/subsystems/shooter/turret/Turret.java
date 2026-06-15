// Copyright (c) DrumTurret project contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root of this project.

package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final Supplier<ChassisSpeeds> drivetrainSpeedsSupplier;

  public Turret(
      TurretIO io,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> drivetrainSpeedsSupplier) {
    this.io = io;
    this.robotPoseSupplier = robotPoseSupplier;
    this.drivetrainSpeedsSupplier = drivetrainSpeedsSupplier;
  }

  public Command aim(Translation2d target) {
    return run(
        () -> {
          var drivetrainSpeeds = drivetrainSpeedsSupplier.get();
          var robotPose = robotPoseSupplier.get();
          var virtualRobotTranslation =
              robotPose
                  .getTranslation()
                  .plus(
                      new Translation2d(
                          drivetrainSpeeds.vxMetersPerSecond, drivetrainSpeeds.vyMetersPerSecond));

          var aimAngleAbsolute = target.minus(virtualRobotTranslation).getAngle();
          var aimAngleRelative = aimAngleAbsolute.minus(robotPose.getRotation());
          io.run(aimAngleRelative.getRadians(), -drivetrainSpeeds.omegaRadiansPerSecond);
        });
  }
}
