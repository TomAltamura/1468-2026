package frc.robot.subsystems.kicker;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Kicker;

public class KickerSubsystem extends SubsystemBase {

  private final TalonFX kickerMotor = new TalonFX(Kicker.KICKER_MOTOR_ID);

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  public KickerSubsystem() {
    // Configure motor
    var config = new com.ctre.phoenix6.configs.TalonFXConfiguration();

    var slot0Configs = new Slot0Configs();
    slot0Configs.kV = 0.35; // Feedforward (Volts per RPS)
    slot0Configs.kA = 0.0;
    slot0Configs.kP = 0.8;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;

    config.Slot0 = slot0Configs;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    kickerMotor.getConfigurator().apply(config);

    //Brake mode for kicker
    kickerMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /**
   * Set the kickwheel velocity in rotations per second (RPS) at the output (wheel) shaft.
   * @param velocityRPS Desired velocity in RPS at the wheel
   */
  public void setVelocity(double velocityRPS) {
    // Convert wheel RPS to motor RPS using gear reduction
    double motorRPS = velocityRPS * Kicker.KICKER_GEAR_RATIO;
    kickerMotor.setControl(velocityRequest.withVelocity(motorRPS));
  }

  public void stop() {
    kickerMotor.stopMotor();
  }
}