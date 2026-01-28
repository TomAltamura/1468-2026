// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // Shooter 
  public static final class Shooter {
      public static final int FLYWHEEL_LEAD_ID = 20;
      public static final int FLYWHEEL_FOLLOWER_ID = 21;
      public static final int HOOD_MOTOR_ID = 22;
  }

  // Kicker
  public static final class Kicker {
    public static final int KICKER_MOTOR_ID = 30;
    public static final double KICKER_TARGET_RPS = 30.0;
    public static final double KICKER_GEAR_RATIO = 1.0; // NEED TO INSERT
  }

  public static final class Harvester {
    public static final int DEPLOY_MOTOR_ID = 40;
    public static final int SPIN_MOTOR_ID = 41;
    public static final double DEPLOY_GEAR_RATIO = 1.0; // NEED TO INSERT
    public static final double SPIN_GEAR_RATIO = 1.0; // NEED TO INSERT
    public static final double DEPLOY_TARGET_POSITION = 10.0; // NEED TO INSERT
    public static final double SPIN_TARGET_RPS = 20.0; // NEED TO INSERT
}

}
