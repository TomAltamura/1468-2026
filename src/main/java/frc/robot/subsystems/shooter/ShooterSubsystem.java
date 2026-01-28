package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX flywheelLead = new TalonFX(20);
  private final TalonFX flywheelFollower = new TalonFX(21);
  private final TalonFX hoodMotor = new TalonFX(22);

  // Create separate requests for every motor
  //   private final DutyCycleOut leadRequest = new DutyCycleOut(0);
  //   private final DutyCycleOut followerRequest = new DutyCycleOut(0);
  private final DutyCycleOut hoodRequest = new DutyCycleOut(0);

  private final VelocityVoltage leadRequest = new VelocityVoltage(0);
  private final VelocityVoltage followerRequest = new VelocityVoltage(0);

  public ShooterSubsystem() {
    // Factory default configurations
    var config = new com.ctre.phoenix6.configs.TalonFXConfiguration();

    var slot0Configs = new Slot0Configs();
    slot0Configs.kV = 0.37; // Feedforward (Volts per RPS) - Critical for velocity
    slot0Configs.kA = 0.43; // Applied voltage
    slot0Configs.kP = 0.89; // Proportional Gain (Error correction)
    slot0Configs.kI = 0.0; // Integral Gain
    slot0Configs.kD = 0.0; // Derivative Gain

    config.Slot0 = slot0Configs;

    // Shooter Motor Config
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    flywheelLead.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    flywheelFollower.getConfigurator().apply(config);

    // Hood Config
    var hoodConfig = new com.ctre.phoenix6.configs.TalonFXConfiguration();
    hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    hoodMotor.getConfigurator().apply(hoodConfig);

    // Set Neutral Modes
    flywheelLead.setNeutralMode(NeutralModeValue.Coast);
    flywheelFollower.setNeutralMode(NeutralModeValue.Coast);
    hoodMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setFlywheel(double velocityRPS) {
    // Control both motors manually
    flywheelLead.setControl(leadRequest.withVelocity(velocityRPS * 100.0));
    flywheelFollower.setControl(followerRequest.withVelocity(velocityRPS * 100.0));
  }

  public void setHood(double percent) {
    hoodMotor.setControl(hoodRequest.withOutput(percent));
  }

  public void stop() {
    flywheelLead.stopMotor();
    flywheelFollower.stopMotor();
    hoodMotor.stopMotor();
  }
}
