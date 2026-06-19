// Copyright (c) DrumTurret project contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the BSD license file in the root of this project.

package frc.robot.subsystems.shooter;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.util.Map;
import java.util.TreeMap;

public class ShotMap {
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

  private final TreeMap<Double, ShotMapEntry> map;
  private final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

  public ShotMap(@JsonProperty("map") TreeMap<Double, ShotMapEntry> map) {
    this.map = map;
    map.forEach((k, v) -> timeOfFlightMap.put(k, v.timeOfFlight));
  }

  public ShotResult get(Double height, Double distance) {
    var val = map.get(height);
    if (val == null) {
      var ceilingKey = map.ceilingKey(height);
      var floorKey = map.floorKey(height);

      if (ceilingKey == null && floorKey == null) {
        return null;
      }
      if (ceilingKey == null) {
        return map.get(floorKey).get(distance);
      }
      if (floorKey == null) {
        return map.get(ceilingKey).get(distance);
      }
      var floor = map.get(floorKey).get(distance);
      var ceiling = map.get(ceilingKey).get(distance);

      return floor.interpolate(ceiling, distance);
    } else {
      return val.get(distance);
    }
  }

  public double getTimeOfFlight(double height) {
    return timeOfFlightMap.get(height);
  }

  public static class ShotMapEntry extends InterpolatingTreeMap<Double, ShotResult> {
    final double timeOfFlight;

    public ShotMapEntry(
        @JsonProperty("tof") double timeOfFlight,
        @JsonProperty("solutions") Map<Double, ShotResult> solutions) {
      super(MathUtil::inverseInterpolate, ShotResult::interpolate);
      this.timeOfFlight = timeOfFlight;
      solutions.forEach(this::put);
    }
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
