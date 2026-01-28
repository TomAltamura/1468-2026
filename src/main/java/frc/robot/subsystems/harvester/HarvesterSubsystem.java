package frc.robot.subsystems.harvester;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Harvester;

public class HarvesterSubsystem extends SubsystemBase {
    private final TalonFX deployMotor = new TalonFX(Harvester.DEPLOY_MOTOR_ID);
    private final TalonFX spinMotor = new TalonFX(Harvester.SPIN_MOTOR_ID);

    private final PositionVoltage deployPositionRequest = new PositionVoltage(0);
    private final VelocityVoltage deployVelocityRequest = new VelocityVoltage(0);
    private final VelocityVoltage spinVelocityRequest = new VelocityVoltage(0);

    public HarvesterSubsystem() {
        // Deploy motor config (position/velocity, brake)
        var deployConfig = new com.ctre.phoenix6.configs.TalonFXConfiguration();
        var deploySlot0 = new Slot0Configs();
        deploySlot0.kP = 1.0;
        deploySlot0.kI = 0.0;
        deploySlot0.kD = 0.0;
        deploySlot0.kV = 0.2;
        deployConfig.Slot0 = deploySlot0;
        deployConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        deployMotor.getConfigurator().apply(deployConfig);
        deployMotor.setNeutralMode(NeutralModeValue.Brake);

        // Spin motor config (velocity, coast)
        var spinConfig = new com.ctre.phoenix6.configs.TalonFXConfiguration();
        var spinSlot0 = new Slot0Configs();
        spinSlot0.kP = 0.5;
        spinSlot0.kI = 0.0;
        spinSlot0.kD = 0.0;
        spinSlot0.kV = 0.2;
        spinConfig.Slot0 = spinSlot0;
        spinConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        spinMotor.getConfigurator().apply(spinConfig);
        spinMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    // Deploy position control (rotations at output)
    public void setDeployPosition(double rotations) {
        double motorRotations = rotations * Harvester.DEPLOY_GEAR_RATIO;
        deployMotor.setControl(deployPositionRequest.withPosition(motorRotations));
    }

    // Deploy velocity control (RPS at output)
    public void setDeployVelocity(double rps) {
        double motorRPS = rps * Harvester.DEPLOY_GEAR_RATIO;
        deployMotor.setControl(deployVelocityRequest.withVelocity(motorRPS));
    }

    // Spin velocity control (RPS at output)
    public void setSpinVelocity(double rps) {
        double motorRPS = rps * Harvester.SPIN_GEAR_RATIO;
        spinMotor.setControl(spinVelocityRequest.withVelocity(motorRPS));
    }

    public void stopDeploy() {
        deployMotor.stopMotor();
    }

    public void stopSpin() {
        spinMotor.stopMotor();
    }
}