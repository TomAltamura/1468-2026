package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shooter;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX flywheelLead = new TalonFX(Shooter.FLYWHEEL_LEAD_ID);
  private final TalonFX flywheelFollower = new TalonFX(Shooter.FLYWHEEL_FOLLOWER_ID);
  private final TalonFX hoodMotor = new TalonFX(Shooter.HOOD_MOTOR_ID);
  private final TalonFX turretMotor = new TalonFX(Shooter.TURRET_MOTOR_ID);

  private final VelocityVoltage leadRequest = new VelocityVoltage(0);
  private final PositionVoltage hoodPositionRequest = new PositionVoltage(0);
  private final VelocityVoltage hoodVelocityRequest = new VelocityVoltage(0);
  private final VelocityVoltage turretVelocityRequest = new VelocityVoltage(0);

  public ShooterSubsystem() {
    // Flywheel config
    var flywheelConfig = new TalonFXConfiguration();
    var slot0Configs = new Slot0Configs();
    slot0Configs.kV = Shooter.SHOOT_kV;
    slot0Configs.kA = Shooter.SHOOT_kA;
    slot0Configs.kP = Shooter.SHOOT_kP;
    slot0Configs.kI = Shooter.SHOOT_kI;
    slot0Configs.kD = Shooter.SHOOT_kD;
    flywheelConfig.Slot0 = slot0Configs;
    flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    flywheelLead.getConfigurator().apply(flywheelConfig);
    flywheelLead.setNeutralMode(NeutralModeValue.Coast);

    // Follower config
    flywheelFollower.setControl(new Follower(flywheelLead.getDeviceID(), MotorAlignmentValue.Opposed));
    flywheelFollower.setNeutralMode(NeutralModeValue.Coast);

    // Hood config (positional PID or velocity, brake mode, soft stops)
    var hoodConfig = new TalonFXConfiguration();
    var hoodSlot0 = new Slot0Configs();
    hoodSlot0.kP = Shooter.HOOD_kP;
    hoodSlot0.kI = Shooter.HOOD_kI;
    hoodSlot0.kD = Shooter.HOOD_kD;
    hoodSlot0.kV = Shooter.HOOD_kV;
    hoodConfig.Slot0 = hoodSlot0;
    hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Shooter.HOOD_TOP_SOFT_LIMIT_ROT;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Shooter.HOOD_BOTTOM_SOFT_LIMIT_ROT;
    hoodMotor.getConfigurator().apply(hoodConfig);
    hoodMotor.setNeutralMode(NeutralModeValue.Brake);

    // Turret config (velocity, brake mode, soft stops)
    var turretConfig = new TalonFXConfiguration();
    var turretSlot0 = new Slot0Configs();
    turretSlot0.kP = Shooter.TURRET_kP;
    turretSlot0.kI = Shooter.TURRET_kI;
    turretSlot0.kD = Shooter.TURRET_kD;
    turretSlot0.kV = Shooter.TURRET_kV;
    turretConfig.Slot0 = turretSlot0;
    turretConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Shooter.TURRET_RIGHT_SOFT_LIMIT_ROT;
    turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Shooter.TURRET_LEFT_SOFT_LIMIT_ROT;
    turretMotor.getConfigurator().apply(turretConfig);
    turretMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    log();
  }

  // Flywheel velocity (RPS at output)
  public void setFlywheel(double velocityRPS) {
    velocityRPS = velocityRPS * 100; // Temporary scaling for testing
    flywheelLead.setControl(leadRequest.withVelocity(velocityRPS));
  }

  // Hood position (rotations at output)
  public void setHoodPosition(double rotations) {
    double motorRotations = rotations * Shooter.HOOD_GEAR_RATIO;
    hoodMotor.setControl(hoodPositionRequest.withPosition(motorRotations));
  }

  // Hood velocity (RPS at output)
  public void setHoodVelocity(double rps) {
    double motorRPS = rps * Shooter.HOOD_GEAR_RATIO;
    hoodMotor.setControl(hoodVelocityRequest.withVelocity(motorRPS));
  }
// Turret position (rotations at output)
  public void setTurretPosition(double rotations) {
  double motorRotations = rotations * Shooter.TURRET_GEAR_RATIO;
  turretMotor.setControl(new PositionVoltage(motorRotations));
}

  // Turret velocity (RPS at output)
  public void setTurretVelocity(double rps) {
    double motorRPS = rps * Shooter.TURRET_GEAR_RATIO;
    turretMotor.setControl(turretVelocityRequest.withVelocity(motorRPS));
  }

  public void stop() {
    flywheelLead.stopMotor();
    flywheelFollower.stopMotor();
    hoodMotor.stopMotor();
    turretMotor.stopMotor();
  }

  public void log() {
    // Flywheel
    SmartDashboard.putNumber("Shooter Lead Velocity (RPS)", flywheelLead.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Follower Velocity (RPS)", flywheelFollower.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Lead Output", flywheelLead.get());
    SmartDashboard.putNumber("Shooter Follower Output", flywheelFollower.get());
    
    // Hood
    SmartDashboard.putNumber("Shooter Hood Position (rot, output)", hoodMotor.getPosition().getValueAsDouble() / Shooter.HOOD_GEAR_RATIO);
    SmartDashboard.putNumber("Shooter Hood Velocity (RPS, output)", hoodMotor.getVelocity().getValueAsDouble() / Shooter.HOOD_GEAR_RATIO);
   
    // Turret
    SmartDashboard.putNumber("Shooter Turret Position (rot, output)", turretMotor.getPosition().getValueAsDouble() / Shooter.TURRET_GEAR_RATIO);
    SmartDashboard.putNumber("Shooter Turret Velocity (RPS, output)", turretMotor.getVelocity().getValueAsDouble() / Shooter.TURRET_GEAR_RATIO);
    }
}