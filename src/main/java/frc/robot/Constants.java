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
      public static final int TURRET_MOTOR_ID = 23;

      //TODO: Gear ratios need to be determined
      public static final double HOOD_GEAR_RATIO = 1.0; 
      public static final double TURRET_GEAR_RATIO = 1.0;  


      //TODO: Tune these values (flywheel)
      public static final double SHOOT_kP = 0.89;
      public static final double SHOOT_kI = 0.0;
      public static final double SHOOT_kD = 0.0;
      public static final double SHOOT_kV = 0.37;
      public static final double SHOOT_kA = 0.43;

      // Hood PID and soft stops (hood)
      public static final double HOOD_kP = 0.5; 
      public static final double HOOD_kI = 0.0;
      public static final double HOOD_kD = 0.0;
      public static final double HOOD_kV = 0.0;
      public static final double HOOD_TOP_SOFT_LIMIT_ROT = 10.0;
      public static final double HOOD_BOTTOM_SOFT_LIMIT_ROT = 0.0; 

      // Turret PID and soft stops (Turret)
      public static final double TURRET_kP = 0.5; 
      public static final double TURRET_kI = 0.0;
      public static final double TURRET_kD = 0.0;
      public static final double TURRET_kV = 0.0;
      public static final double TURRET_RIGHT_SOFT_LIMIT_ROT = 5.0;
      public static final double TURRET_LEFT_SOFT_LIMIT_ROT = -5.0; 

  }

  // Kicker
  public static final class Kicker {
    public static final int KICKER_MOTOR_ID = 30;
    public static final double KICKER_TARGET_RPS = 30.0;
    public static final double KICKER_GEAR_RATIO = 1.0; // NEED TO INSERT

    //TODO: Tune these values
    public static final double kP = 0.89;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.37;
    public static final double kA = 0.43;
  }

  // Harvester
  public static final class Harvester {
    public static final int DEPLOY_MOTOR_ID = 40;
    public static final int SPIN_MOTOR_ID = 41;
    public static final double DEPLOY_GEAR_RATIO = 1.0; // NEED TO INSERT
    public static final double SPIN_GEAR_RATIO = 1.0; // NEED TO INSERT
    public static final double DEPLOY_TARGET_POSITION = 10.0; // NEED TO INSERT (position in rotations)
    public static final double SPIN_TARGET_RPS = 20.0; // NEED TO INSERT

    //TODO: Tune these values
    public static final double deploykP = 0.89;
    public static final double deploykI = 0.0;
    public static final double deploykD = 0.0;
    public static final double deploykV = 0.37;

    public static final double spinkP = 0.89;
    public static final double spinkI = 0.0;
    public static final double spinkD = 0.0;
    public static final double spinkV = 0.37;

  }

  // Indexer
  public static final class Indexer {
      public static final int MOTOR_ID = 50;
      public static final double GEAR_RATIO = 1.0; // NEED TO INSERT
      public static final double TARGET_RPS = 10.0; // NEED TO INSERT
      public static final double[] POSITIONS = {0.0, 2.0, 4.0, 6.0}; // NEED TO INSERT (positions in rotations)

      //TODO: Tune these values
      public static final double kP = 0.89;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double kV = 0.37;
  }

  //Climber
  public static final class Climber {
    public static final int LEFT_MOTOR_ID = 60;
    public static final int RIGHT_MOTOR_ID = 61;

    public static final double FORWARD_SOFT_LIMIT_ROT = 20.0; // rotations
    public static final double REVERSE_SOFT_LIMIT_ROT = 0.0;  // rotations
    public static final double UP_SPEED = 0.5;
    public static final double DOWN_SPEED = -0.5;
    public static final double SMALL_MOVE_ROT = 0.5;
    public static final double HOME_POSITION_ROT = 0.0;

    //TODO: Tune these values
    public static final double kP = 0.89;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.37;
}


}
