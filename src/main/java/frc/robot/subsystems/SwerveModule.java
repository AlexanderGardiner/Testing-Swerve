// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants.ModuleConstants;
import frc.robot.util.PIDConfig;
import frc.robot.util.SparkMax.SparkMaxConfig;
import frc.robot.util.SparkMax.SparkMaxEncoderType;
import frc.robot.util.SparkMax.SparkMaxSetup;
import frc.robot.util.SparkMax.SparkMaxStatusFrames;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

  private final TalonFX driveMotor;
  TalonFXConfiguration driveConfig = new TalonFXConfiguration();

  private final CANSparkMax turningMotor;
  private SparkMaxConfig turningConfig;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel      The channel of the drive motor.
   * @param turningMotorChannel    The channel of the turning motor.
   * @param driveEncoderReversed   Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed) {
    driveMotor = new TalonFX(driveMotorChannel);
    driveMotor.getConfigurator().apply(new TalonFXConfiguration());
    driveConfig.Slot0.kS = 0;
    driveConfig.Slot0.kV = 0.05;
    driveConfig.Slot0.kP = 0.05;
    driveConfig.Slot0.kI = 0;
    driveConfig.Slot0.kD = 0;
    driveConfig.CurrentLimits.SupplyCurrentLimit = 30;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentThreshold = 60;
    driveConfig.CurrentLimits.SupplyTimeThreshold = 2;
    driveConfig.Feedback.SensorToMechanismRatio = 11.0 / 40.0;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveMotor.getConfigurator().apply(driveConfig);
    turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
    turningConfig = new SparkMaxConfig(
        new SparkMaxStatusFrames(
            500,
            30,
            500,
            500,
            500,
            20,
            500),
        1000, true, SparkMaxEncoderType.Absolute, IdleMode.kBrake, 30, 30, false, false, 1, true,
        new PIDConfig(5, 0, 0));
    SparkMaxSetup.setup(turningMotor, turningConfig);

  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveMotor.getVelocity().getValueAsDouble(),
        new Rotation2d(2 * Math.PI * turningMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveMotor.getPosition().getValueAsDouble(),
        new Rotation2d(2 * Math.PI * turningMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition()));

  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(
        2 * Math.PI * (turningMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition() / 360));
    SmartDashboard.putNumber("encoder rotaiton", encoderRotation.getDegrees());
    SwerveModuleState state = SwerveModuleState.optimize(desiredState,
        encoderRotation);

    // state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    VelocityVoltage driveVelocity = new VelocityVoltage(state.speedMetersPerSecond / (Math.PI * .0762));
    driveMotor.setControl(driveVelocity);
    SmartDashboard.putNumber("module", state.speedMetersPerSecond / (Math.PI * .0762));
    SmartDashboard.putNumber("angle123", driveMotor.getVelocity().getValueAsDouble());
    turningMotor.getPIDController().setReference(Math.abs(state.angle.getRadians() / (2 * Math.PI) - .5),
        ControlType.kPosition);
  }
}
