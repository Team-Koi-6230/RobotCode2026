// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.*;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure.WantedState;

public class ClimberSubsystem extends SubsystemBase {
  public enum ClimberState {
    MOVING_GROUND,
    AT_TARGET_GROUND,
    MOVING_HANG,
    AT_TARGET_HANG
  }

  


  private double targetHeight = 0.0;
  private ClimberState state = ClimberState.MOVING_GROUND;
  private WantedState currentWantedState;
  private final SparkMax m_motor;
  private final SparkMax s_motor;
  private final RelativeEncoder encoder;
  private final DutyCycleEncoder abs_encoder;
  private final SparkClosedLoopController closedLoop;
  private boolean isGrounded = true;
  private boolean hasExtendedL1 = false;

  public ClimberSubsystem() {
    m_motor = new SparkMax(Constants.ClimberConstants.kMainMotorID, MotorType.kBrushless);
    s_motor = new SparkMax(Constants.ClimberConstants.kSecondaryMotorID, MotorType.kBrushless);
    encoder = m_motor.getEncoder();
    abs_encoder = new DutyCycleEncoder(Constants.ClimberConstants.kDutyCycleChannel,
        Constants.ClimberConstants.kMetersPerRotation, Constants.ClimberConstants.kDutyCycleOffset);
    closedLoop = m_motor.getClosedLoopController();
    SparkMaxConfig config = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();

    // slot 0 is for ground and clawed, slot 1 is for when the entire robot's mass
    // is on the climber elevator.
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(
            Constants.ClimberConstants.kP_ground,
            Constants.ClimberConstants.kI_ground,
            Constants.ClimberConstants.kD_ground,
            ClosedLoopSlot.kSlot0).feedForward
        .kS(Constants.ClimberConstants.kS_ground, ClosedLoopSlot.kSlot0)
        .kG(Constants.ClimberConstants.kG_ground, ClosedLoopSlot.kSlot0)
        .kV(Constants.ClimberConstants.kV_ground, ClosedLoopSlot.kSlot0)
        .kA(Constants.ClimberConstants.kA_ground, ClosedLoopSlot.kSlot0);

    config.closedLoop
        .pid(
            Constants.ClimberConstants.kP_hang,
            Constants.ClimberConstants.kI_hang,
            Constants.ClimberConstants.kD_hang,
            ClosedLoopSlot.kSlot1).feedForward
        .kS(Constants.ClimberConstants.kS_hang, ClosedLoopSlot.kSlot1)
        .kG(Constants.ClimberConstants.kG_hang, ClosedLoopSlot.kSlot1)
        .kV(Constants.ClimberConstants.kV_hang, ClosedLoopSlot.kSlot1)
        .kA(Constants.ClimberConstants.kA_hang, ClosedLoopSlot.kSlot1);

    config.closedLoop.maxMotion
        .allowedProfileError(Constants.ClimberConstants.kTolerance)
        .maxAcceleration(Constants.ClimberConstants.kMaxAcceleration)
        .cruiseVelocity(Constants.ClimberConstants.kMaxVelocity)
        .allowedProfileError(Constants.ClimberConstants.kTolerance, ClosedLoopSlot.kSlot1)
        .maxAcceleration(Constants.ClimberConstants.kMaxAcceleration, ClosedLoopSlot.kSlot1)
        .cruiseVelocity(Constants.ClimberConstants.kMaxVelocity, ClosedLoopSlot.kSlot1);

    config
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.ClimberConstants.kStallLimit);
    config.encoder
        .positionConversionFactor(Constants.ClimberConstants.kMetersPerRotation)
        .velocityConversionFactor(Constants.ClimberConstants.kMetersPerRotation / 60.0);

    m_motor.configure(config, com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);
    followerConfig.follow(m_motor, true);
    s_motor.configure(followerConfig, com.revrobotics.ResetMode.kResetSafeParameters,
        com.revrobotics.PersistMode.kPersistParameters);

    encoder.setPosition(abs_encoder.get());
  }

  public Command setHeightCommandGround(double height) {
    return runOnce(() -> {
      setPositionGround(height);
    });
  }

  public Command setHeightCommandHang(double height) {
    return runOnce(() -> {
      setPositionHang(height);
    });
  }

  public Command extendClimberGroundCommand() {
    return setHeightCommandGround(Constants.ClimberConstants.kL1ExtendHeight);
  }

  public Command extendClimberHangCommand() {
    return setHeightCommandHang(Constants.ClimberConstants.kL1ExtendHeight);
  }

  public Command closeClimberGroundCommand() {
    return setHeightCommandGround(Constants.ClimberConstants.kL1CloseHeight);
  }

  public Command closeClimberHangCommand() {
    return setHeightCommandHang(Constants.ClimberConstants.kL1CloseHeight);
  }

  public void stop() {
    m_motor.stopMotor();
  }

  public ClimberState getState() {
    return state;
  }

  public double getHeight() {
    return encoder.getPosition();
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  public void setPositionGround(double targetHeight) {
    this.targetHeight = targetHeight;
    isGrounded = true;
  }

  public void setPositionHang(double targetHeight) {
    this.targetHeight = targetHeight;
    isGrounded = false;
  }

  @Override
  public void periodic() {
    handleState();
    motorLogic();
  }

  public boolean isReady() {
    return true;
  }

  private void HandleL1() {
    if (!hasExtendedL1) {
    setPositionGround(Constants.ClimberConstants.kL1ExtendHeight);

    if (state == ClimberState.AT_TARGET_GROUND) {
      hasExtendedL1 = true;
    }

  } else {
    setPositionHang(Constants.ClimberConstants.kL1CloseHeight);
  }
  }

  private void handleState() {
    if (currentWantedState != null) {
      switch (currentWantedState) {
        case IDLE:
        case HOME:
        case SHOOTING:
        case PREPARING_SHOOTER:
        case PREPARING_SHOOTER_AND_INTAKING:
        case SHOOTING_AND_INTAKING:
          setHeightCommandGround(0);
          break;
        case L1_EXTEND_TELEOP: // WORK HERE!!!!!!!!!!
          setPositionGround(Constants.ClimberConstants.kL1ExtendTeleop);
          break;
        case L1_CLOSE_TELEOP: 
          setPositionHang(Constants.ClimberConstants.kL1CloseTeleop);
          break;
        case L1_EXTEND_AUTON:
          setPositionGround(Constants.ClimberConstants.kL1ExtendAuton);
          break;
        case L1_CLOSE_AUTON:
          setPositionHang(Constants.ClimberConstants.kL1CloseAuton);
          break;
          default:
          break;
      }
    }
  }

  private void motorLogic() {
    if (isGrounded) {
      closedLoop.setSetpoint(targetHeight, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
      if (Math.abs(targetHeight - getHeight()) < Constants.ClimberConstants.kTolerance) {
        state = ClimberState.AT_TARGET_GROUND;
      } else {
        state = ClimberState.MOVING_GROUND;
      }
    } else {
      closedLoop.setSetpoint(targetHeight, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot1);
      if (Math.abs(targetHeight - getHeight()) < Constants.ClimberConstants.kTolerance) {
        state = ClimberState.AT_TARGET_HANG;
      } else {
        state = ClimberState.MOVING_HANG; 
      }
    }
  }

  public void setWantedState(WantedState wantedState) {
    this.currentWantedState = wantedState;
    if (wantedState != WantedState.L1_EXTEND_TELEOP) {
    hasExtendedL1 = false;
  }
  }
}