// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.Notifier;
import java.util.function.Supplier;

public class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  /** Signals for synchronized refresh. */
  private static BaseStatusSignal[] drivebaseSignals = new BaseStatusSignal[0];

  private static BaseStatusSignal[] superstructureSignals = new BaseStatusSignal[0];

  /** Notifier loop for signal refresh */
  private static final Notifier signalThread = new Notifier(PhoenixUtil::waitForAll);

  /** Registers a set of signals for synchronized refresh. */
  public static void registerSignals(CANBus canbus, BaseStatusSignal... signals) {
    if (canbus.getName().equals("Drivebase")) {
      BaseStatusSignal[] newSignals =
          new BaseStatusSignal[drivebaseSignals.length + signals.length];
      System.arraycopy(drivebaseSignals, 0, newSignals, 0, drivebaseSignals.length);
      System.arraycopy(signals, 0, newSignals, drivebaseSignals.length, signals.length);
      drivebaseSignals = newSignals;
    } else {
      BaseStatusSignal[] newSignals =
          new BaseStatusSignal[superstructureSignals.length + signals.length];
      System.arraycopy(superstructureSignals, 0, newSignals, 0, superstructureSignals.length);
      System.arraycopy(signals, 0, newSignals, superstructureSignals.length, signals.length);
      superstructureSignals = newSignals;
    }
  }

  /** Refresh all registered signals. */
  public static void waitForAll() {
    if (drivebaseSignals.length > 0) {
      BaseStatusSignal.waitForAll(0.02, drivebaseSignals);
    }
    if (superstructureSignals.length > 0) {
      BaseStatusSignal.waitForAll(0.02, superstructureSignals);
    }
  }

  /** Start a thread for refreshing signals */
  public static void startTelemetry() {
    signalThread.startPeriodic(0.02);
  }
}
