// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.*;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
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

  public enum L3ClimbJourney {
    NONE,
    L1Open,
    L1Closed,
    L2Open,
    L2Closed,
    L3Open,
    L3Closed
  }

  private L3ClimbJourney L3Journey = L3ClimbJourney.NONE;
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
  private final SysIdRoutine m_SysIdRoutine;
  private final MutVoltage m_appliedVoltage;
    private final MutAngle m_angle;
    private final MutAngularVelocity m_velocity;

  public ClimberSubsystem() {
    m_motor = new SparkMax(Constants.ClimberConstants.kMainMotorID, MotorType.kBrushless);
    s_motor = new SparkMax(Constants.ClimberConstants.kSecondaryMotorID, MotorType.kBrushless);
    encoder = m_motor.getEncoder();
    abs_encoder = new DutyCycleEncoder(Constants.ClimberConstants.kDutyCycleChannel,
        Constants.ClimberConstants.kMetersPerRotation, Constants.ClimberConstants.kDutyCycleOffset);
    closedLoop = m_motor.getClosedLoopController();
    SparkMaxConfig config = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();
    
    //SysId
    m_angle = Radians.mutable(Constants.ClimberConstants.kMutTragetAngle);
    m_appliedVoltage = Volts.mutable(Constants.ClimberConstants.kMutVolts);
    m_velocity = RadiansPerSecond.mutable(Constants.ClimberConstants.kMutVelocity);

    m_SysIdRoutine = new SysIdRoutine(new Config(), new Mechanism(
            m_motor::setVoltage,
             log -> {
                // Record a frame for the intake motor.
                log.motor("IntakeArm")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            m_motor.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(encoder.getPosition(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(encoder.getVelocity(), RotationsPerSecond));
              }, this
            ));

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

    config.idleMode(IdleMode.kBrake);
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

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  return m_SysIdRoutine.quasistatic(direction);
}

public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  return m_SysIdRoutine.dynamic(direction);
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
    setPositionGround(Constants.ClimberConstants.kL1ExtendHeight);
    if (state == ClimberState.AT_TARGET_GROUND) {
      hasExtendedL1 = true;
    }
    if (hasExtendedL1) {
      setPositionHang(Constants.ClimberConstants.kL1CloseHeight);
    }
  }

  private void HandleL3() {
    switch (L3Journey) {
      case NONE:
        setPositionGround(Constants.ClimberConstants.kL1ExtendHeight);
        break;
      case L1Open:
        setPositionHang(Constants.ClimberConstants.kL1CloseHeight);
        break;
      case L1Closed:
        setPositionHang(Constants.ClimberConstants.kL2ExtendHeight);
        break;
      case L2Open:
        setPositionHang(Constants.ClimberConstants.kL2CloseHeight);
        break;
      case L3Closed:
        setPositionHang(Constants.ClimberConstants.kL3ExtendHeight);
        break;
      default:
        break;
    }
    if (L3Journey == L3ClimbJourney.NONE || L3Journey == L3ClimbJourney.L1Closed
        || L3Journey == L3ClimbJourney.L2Closed) {
      if (state == ClimberState.AT_TARGET_GROUND) {
        L3Journey = L3ClimbJourney.values()[L3Journey.ordinal() + 1];
      }
    }
    if (L3Journey == L3ClimbJourney.L1Open || L3Journey == L3ClimbJourney.L2Open
        || L3Journey == L3ClimbJourney.L3Open) {
      if (state == ClimberState.AT_TARGET_GROUND) {
        L3Journey = L3ClimbJourney.values()[L3Journey.ordinal() + 1];
      }
    }

  }

  private void handleState() {
    if (currentWantedState != null) {
      switch (currentWantedState) {
        case IDLE:
        case HOME:
        case SHOOTING:
        case PREPARING_SHOOTER:
          setHeightCommandGround(0);
          break;
        case L1_CLIMB:
          HandleL1();
          break;
        case L3_CLIMB:
          HandleL3();
          break;
        default:
          break;
      }
    }
  }

  private void motorLogic() {
    if (isGrounded) {
      closedLoop.setSetpoint(targetHeight, ControlType.kPosition, ClosedLoopSlot.kSlot0);
      if (Math.abs(targetHeight - getHeight()) < Constants.ClimberConstants.kTolerance) {
        state = ClimberState.AT_TARGET_GROUND;
      } else {
        state = ClimberState.MOVING_GROUND;
      }
    } else {
      closedLoop.setSetpoint(targetHeight, ControlType.kPosition, ClosedLoopSlot.kSlot1);
      if (Math.abs(targetHeight - getHeight()) < Constants.ClimberConstants.kTolerance) {
        state = ClimberState.AT_TARGET_HANG;
      } else {
        state = ClimberState.MOVING_GROUND;
      }
    }
  }

  public void setWantedState(WantedState wantedState) {
    this.currentWantedState = wantedState;
  }
}