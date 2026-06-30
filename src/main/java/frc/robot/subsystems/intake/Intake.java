package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.io.IntakeIO;
import frc.robot.subsystems.intake.io.IntakeIOInputsAutoLogged;
import frc.robot.subsystems.intake.io.IntakeIORev;
import frc.robot.subsystems.intake.io.IntakeIOSim;
import frc.robot.util.roller.Roller;
import frc.robot.util.roller.RollerConfig;
import frc.robot.util.roller.RollerConfig.RollerMotor;
import frc.robot.util.roller.io.RollerIO;
import frc.robot.util.roller.io.RollerIOInputsAutoLogged;
import team6230.koiupstream.subsystems.ConditionalAction;
import team6230.koiupstream.subsystems.ExtraIO;
import team6230.koiupstream.subsystems.UpstreamSubsystem;
import team6230.koiupstream.superstates.Superstate;
import team6230.koiupstream.tunable.Tunable;
import team6230.koiupstream.tunable.TunableManager;

public class Intake extends UpstreamSubsystem<RobotState, IntakeIO, IntakeIOInputsAutoLogged> {
    private final RollerIO roller;
    private final RollerIOInputsAutoLogged rollerInputs;

    private int step = 1;
    private boolean isShooting = false;

    private double oscillationStartTime = -1;

    // #region tunables
    @Tunable
    private double tunablePivotSetpoint = IntakeConstants.kMinAngle.getDegrees();

    @Tunable
    private double tunableRollerVolts = 0;

    @Tunable
    private double kP = Robot.isReal() ? IntakeConstants.kP : IntakeConstants.kPsim;

    @Tunable
    private double kI = Robot.isReal() ? IntakeConstants.kI : IntakeConstants.kIsim;

    @Tunable
    private double kD = Robot.isReal() ? IntakeConstants.kD : IntakeConstants.kDsim;

    @Tunable
    private double kS = Robot.isReal() ? IntakeConstants.kS : IntakeConstants.kSsim;

    @Tunable
    private double kG = Robot.isReal() ? IntakeConstants.kG : IntakeConstants.kGsim;

    @Tunable
    private double kV = Robot.isReal() ? IntakeConstants.kV : IntakeConstants.kVsim;

    @Tunable
    private double kA = Robot.isReal() ? IntakeConstants.kA : IntakeConstants.kAsim;

    @Tunable
    private double kMaxVelocityRadPerSec = Robot.isReal() ? IntakeConstants.kCruiseVelocity
            : IntakeConstants.kMaxVelocityRadPerSec;

    @Tunable
    private double kMaxAccelRadPerSecSquared = Robot.isReal() ? IntakeConstants.kMaxAcceleration
            : IntakeConstants.kMaxAccelRadPerSecSquared;
    // #endregion

    public Intake() {
        super("Intake", new IntakeIOInputsAutoLogged());

        RollerConfig config = new RollerConfig();
        config.motor = RollerMotor.NEO;
        config.name = "IntakeRoller";
        config.motorId = RobotMap.CanBus.kIntakeRollerID;
        config.gearRatio = IntakeConstants.kRollerGearRatio;
        config.smartCurrentLimit = 80;
        roller = Roller.makeRollerIO(config);

        rollerInputs = new RollerIOInputsAutoLogged();

        addAnExtraIO(new ExtraIO(roller, rollerInputs, "Roller"));

        addSuperstateBehaviour(RobotState.INTAKING, () -> intaking());
        addSuperstateBehaviour(RobotState.PREPARING_SHOOTER_AND_INTAKING, () -> intaking());
        addSuperstateBehaviour(RobotState.SHOOTING_AND_INTAKING, () -> intaking());
        addSuperstateBehaviour(RobotState.UNJAM, () -> {
            clearConditionalActions();
            roller.runVoltage(-IntakeConstants.kIntakingVolts);
        });
        addSuperstateBehaviour(RobotState.IDLE, () -> {
            clearConditionalActions();
            roller.runVoltage(0);
            isShooting = false;
        });
        addSuperstateBehaviour(RobotState.PREPARING_SHOOTER, () -> {
            clearConditionalActions();
            roller.runVoltage(0);
            isShooting = false;
        });
        addSuperstateBehaviour(RobotState.SHOOTING, () -> {
            roller.runVoltage(IntakeConstants.kShootingVolts);
            oscillating();
        });
        addSuperstateBehaviour(RobotState.PRESHOOTING, () -> openArm());
        addSuperstateBehaviour(RobotState.HOME, () -> home());
    }

    private void home() {
        clearConditionalActions();
        io.setTargetAngle(IntakeConstants.kClosedAngle);
        roller.runVoltage(0);
        isShooting = false;
    }

    private void intaking() {
        clearConditionalActions();
        io.setTargetAngle(IntakeConstants.kOpenAngle);
        registerConditionalAction(new ConditionalAction(
                () -> inputs.relativePivotAngleDeg < IntakeConstants.kMinOpenAngle.getDegrees(),
                () -> roller.runVoltage(IntakeConstants.kIntakingVolts)));
        isShooting = false;
    }

    private void openArm() {
        clearConditionalActions();
        io.setTargetAngle(IntakeConstants.kOpenAngle);
        roller.runVoltage(0);
    }

    private void shooting() {
        if (isShooting)
            return;
        clearConditionalActions();
        io.setTargetAngle(IntakeConstants.kOpenAngle);
        isShooting = true;
        step = 1;

        registerConditionalAction(new ConditionalAction(
                this::shouldAdvanceIndexCycle,
                this::advanceIndexCycle));
    }

    private void oscillating() {
        clearConditionalActions();
        oscillationStartTime = Timer.getFPGATimestamp();

        registerConditionalAction(new ConditionalAction(
                () -> {
                    double elapsed = Timer.getFPGATimestamp() - oscillationStartTime;
                    long cycleIndex = (long) (elapsed / IntakeConstants.kOscillationIntervalSecs);
                    return cycleIndex % 2 != 0;
                },
                () -> {
                    io.setTargetAngle(IntakeConstants.kClosedAngle);
                    oscillationStartTime = Timer.getFPGATimestamp();
                }));
    }

    private boolean shouldAdvanceIndexCycle() {
        return inputs.relativePivotAngleDeg < IntakeConstants.kMinOpenAngle.getDegrees();
    }

    private void advanceIndexCycle() {
        roller.runVoltage(IntakeConstants.kShootingVolts);

        double angleDeg = IntakeConstants.kMinAngle.getDegrees() + (step * IntakeConstants.kStepSizeDegrees);

        if (angleDeg > IntakeConstants.kMaxAngle.getDegrees()) {
            step = 1;
            io.setTargetAngle(IntakeConstants.kOpenAngle);
            return;
        }

        if (inputs.relativePivotAngleDeg <= inputs.pivotTargetAngle - 3) {
            return;
        }

        step++;
        io.setTargetAngle(Rotation2d.fromDegrees(angleDeg));
    }

    @Override
    public void update() {
        if (Superstate.getInstance().isCurrentWanted(RobotState.SHOOTING)) {
            double elapsed = Timer.getFPGATimestamp() - oscillationStartTime;
            long cycleIndex = (long) (elapsed / IntakeConstants.kOscillationIntervalSecs);
            if (cycleIndex % 2 == 0) {
                io.setTargetAngle(IntakeConstants.kOpenAngle);
            } else {
                io.setTargetAngle(IntakeConstants.kClosedAngle);
            }
        }

        if (TunableManager.checkChanged(this)) {
            io.setPIDF(kP, kI, kD, kS, kG, kV, kA, kMaxVelocityRadPerSec, kMaxAccelRadPerSecSquared);
            io.setTargetAngle(Rotation2d.fromDegrees(tunablePivotSetpoint));
            roller.runVoltage(tunableRollerVolts);
        }
    }

    @Override
    public boolean isReady() {
        if (Superstate.getInstance().isCurrentWanted(RobotState.INTAKING)
                || Superstate.getInstance().isCurrentWanted(RobotState.HOME))
            return Math.abs(inputs.pivotTargetAngle
                    - inputs.relativePivotAngleDeg) <= IntakeConstants.kErrorToleranceDeg;
        return true;
    }

    @Override
    public IntakeIO getIO() {
        switch (Constants.currentMode) {
            case REAL:
                return new IntakeIORev();

            case SIM:
                return new IntakeIOSim();

            case REPLAY:
                return new IntakeIO() {
                };
        }
        return new IntakeIO() {
        };
    }

    public void setVoltage(double volts) {
        io.runVoltsPivot(volts);
    }

    public Rotation2d getPosition() {
        return Rotation2d.fromDegrees(inputs.relativePivotAngleDeg);
    }

    public void stopMotor() {
        io.stop();
    }

    public Command intakeManual() {
        return run(() -> {
            setVoltage(-3);
            roller.runVoltage(0/* IntakeConstants.kIntakingVolts */);
        });
    }

    public Command cancel() {
        return run(() -> {
            setVoltage(3);
            roller.runVoltage(0);
        });
    }
}