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
import frc.robot.Constants.Climber;

public class ClimberSubsystem extends SubsystemBase {

  private final TalonFX leftClimberMotor = new TalonFX(Climber.LEFT_MOTOR_ID);
  private final TalonFX rightClimberMotor = new TalonFX(Climber.RIGHT_MOTOR_ID);

  private final PositionVoltage positionRequest = new PositionVoltage(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  public ClimberSubsystem() {
    // Configure left motor
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = Climber.kP;
    slot0.kI = Climber.kI;
    slot0.kD = Climber.kD;
    slot0.kV = Climber.kV;
    config.Slot0 = slot0;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Climber.FORWARD_SOFT_LIMIT_ROT;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Climber.REVERSE_SOFT_LIMIT_ROT;
    leftClimberMotor.getConfigurator().apply(config);
    leftClimberMotor.setNeutralMode(NeutralModeValue.Brake);

    // Configure right motor to follow left
    rightClimberMotor.setControl(
        new Follower(leftClimberMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    rightClimberMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    log();
  }

  public void setCoastMode() {
    leftClimberMotor.setNeutralMode(NeutralModeValue.Coast);
    rightClimberMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setBrakeMode() {
    leftClimberMotor.setNeutralMode(NeutralModeValue.Brake);
    rightClimberMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setMotorSpeed(double speed) {
    leftClimberMotor.set(speed);
  }

  public void climberUp() {
    leftClimberMotor.set(Climber.UP_SPEED);
  }

  public void climberDown() {
    leftClimberMotor.set(Climber.DOWN_SPEED);
  }

  public void climberStop() {
    leftClimberMotor.set(0);
  }

  public void setPosition(double rotations) {
    leftClimberMotor.setControl(positionRequest.withPosition(rotations));
  }

  public void setVelocity(double rps) {
    leftClimberMotor.setControl(velocityRequest.withVelocity(rps));
  }

  public double getEncoderRotations() {
    return leftClimberMotor.getPosition().getValueAsDouble();
  }

  public double getSmallUpLocation() {
    double temp = getEncoderRotations();
    if ((Climber.FORWARD_SOFT_LIMIT_ROT - temp) > Climber.SMALL_MOVE_ROT)
      return (temp + Climber.SMALL_MOVE_ROT);
    else return (Climber.FORWARD_SOFT_LIMIT_ROT - 0.1);
  }

  public double getSmallDownLocation() {
    double temp = getEncoderRotations();
    if (temp > Climber.SMALL_MOVE_ROT) return (temp - Climber.SMALL_MOVE_ROT);
    else return Climber.HOME_POSITION_ROT;
  }

  public boolean isAtTop() {
    return (leftClimberMotor.get() > 0.1)
        && (getEncoderRotations() > Climber.FORWARD_SOFT_LIMIT_ROT);
  }

  public boolean isAtBot() {
    return (leftClimberMotor.get() < -0.1)
        && (getEncoderRotations() < (Climber.REVERSE_SOFT_LIMIT_ROT + 0.125));
  }

  public void log() {
    SmartDashboard.putNumber("Climber Encoder Rot", getEncoderRotations());
    SmartDashboard.putNumber("Climber Speed", leftClimberMotor.get());
    SmartDashboard.putNumber("Climber Lt motor temperature", getLeftClimberTemp());
    SmartDashboard.putNumber("Climber Rt motor temperature", getRightClimberTemp());
    SmartDashboard.putBoolean("Climber Is At Top", isAtTop());
    SmartDashboard.putBoolean("Climber Is At Bot", isAtBot());
  }

  public double getLeftClimberTemp() {
    return leftClimberMotor.getDeviceTemp().getValueAsDouble();
  }

  public double getRightClimberTemp() {
    return rightClimberMotor.getDeviceTemp().getValueAsDouble();
  }
}
