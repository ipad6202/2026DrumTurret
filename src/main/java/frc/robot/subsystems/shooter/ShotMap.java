// Copyright (c) DrumTurret project contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root of this project.

package frc.robot.subsystems.shooter;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.util.Map;

public class ShotMap extends InterpolatingTreeMap<Double, ShotMap.ShotResult> {
  public static ShotMap loadFromDeploy(String filePath) {
    try {
      System.out.println(
          "Loading shot map from " + Filesystem.getDeployDirectory() + File.separator + filePath);
      var mapper = new ObjectMapper();
      mapper.configure(JsonParser.Feature.INCLUDE_SOURCE_IN_LOCATION, true);
      var map =
          mapper.readValue(
              new File(Filesystem.getDeployDirectory() + File.separator + filePath), ShotMap.class);
      System.out.println("Loaded shot map");
      return map;
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  public final double timeOfFlight;

  public ShotMap(
      @JsonProperty("tof") double timeOfFlight,
      @JsonProperty("solutions") Map<Double, ShotResult> solutions) {
    super(MathUtil::inverseInterpolate, ShotResult::interpolate);
    this.timeOfFlight = timeOfFlight;
    solutions.forEach(this::put);
  }

  public record ShotResult(
      @JsonProperty("pitchRad") double pitchRad,
      @JsonProperty("speedMetersPerSec") double speedMetersPerSec) {
    public ShotResult interpolate(ShotResult endValue, double t) {
      return new ShotResult(
          MathUtil.interpolate(pitchRad, endValue.pitchRad, t),
          MathUtil.interpolate(speedMetersPerSec, endValue.speedMetersPerSec, t));
    }
  }
}
