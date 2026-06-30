package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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

    /**
     * Which hard stop the pivot is currently being driven toward while SHOOTING.
     */
    private enum OscillationTarget {
        OPEN, CLOSED
    }

    private OscillationTarget oscillationTarget = OscillationTarget.OPEN;
    private double oscillationStartTime = -1;

    // Reset (not just .calculate(false)'d) on every new pivot target so a fresh
    // kStallDebounceSec window is required before "stalled" can fire again. This is
    // used for
    // arrival detection only (INTAKING/HOME) - the SHOOTING oscillation uses a
    // fixed timing
    // interval instead, since agitating the fuel benefits from a quick, predictable
    // shake
    // rather than waiting for a hard-stop stall on every swing.
    private Debouncer pivotStallDebouncer = newStallDebouncer();

    // Separate, shorter-debounced check for starting the roller during intaking.
    // Kept distinct
    // from pivotStallDebouncer so roller-start can be tuned fast without loosening
    // the
    // isReady() arrival signal that other subsystems gate on.
    private Debouncer rollerStartStallDebouncer = newRollerStartStallDebouncer();

    private static Debouncer newStallDebouncer() {
        return new Debouncer(IntakeConstants.kStallDebounceSec, Debouncer.DebounceType.kRising);
    }

    private static Debouncer newRollerStartStallDebouncer() {
        return new Debouncer(IntakeConstants.kRollerStartStallDebounceSec, Debouncer.DebounceType.kRising);
    }

    // #region tunables
    @Tunable
    private double tunablePivotVolts = 0;

    @Tunable
    private double tunableRollerVolts = 0;
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
        });
        addSuperstateBehaviour(RobotState.PREPARING_SHOOTER, () -> {
            clearConditionalActions();
            roller.runVoltage(0);
        });
        addSuperstateBehaviour(RobotState.SHOOTING, () -> {
            roller.runVoltage(IntakeConstants.kShootingVolts);
            oscillating();
        });
        addSuperstateBehaviour(RobotState.PRESHOOTING, () -> openArm());
        addSuperstateBehaviour(RobotState.HOME, () -> home());
    }

    /**
     * Voltage/current-based replacement for position-tolerance arrival checking.
     * The pivot's
     * meaningful setpoints (open/closed) are mechanical hard stops, so "arrived" is
     * read as
     * "commanding real voltage but current is pinned near the smart current limit,
     * for a
     * sustained period" rather than from any encoder position.
     */
    private boolean isPivotStalled(Debouncer debouncer) {
        double currentRatio = Math.abs(inputs.pivotCurrent[0]) / IntakeConstants.kPivotCurrentLimits;
        double voltageRatio = Math.abs(inputs.pivotAppliedVoltage) / IntakeConstants.kPivotDriveVolts;
        boolean raw = currentRatio >= IntakeConstants.kStallCurrentRatio
                && voltageRatio >= IntakeConstants.kStallVoltageRatio;
        return debouncer.calculate(raw);
    }

    private void home() {
        clearConditionalActions();
        io.runVoltsPivot(IntakeConstants.kPivotClosedVolts);
        roller.runVoltage(0);
        pivotStallDebouncer = newStallDebouncer();
    }

    private void intaking() {
        clearConditionalActions();
        io.runVoltsPivot(IntakeConstants.kPivotOpenVolts);
        pivotStallDebouncer = newStallDebouncer();
        rollerStartStallDebouncer = newRollerStartStallDebouncer();
        roller.runVoltage(0);

        registerConditionalAction(new ConditionalAction(
                () -> isPivotStalled(rollerStartStallDebouncer),
                () -> roller.runVoltage(IntakeConstants.kIntakingVolts)));
    }

    private void openArm() {
        clearConditionalActions();
        io.runVoltsPivot(IntakeConstants.kPivotOpenVolts);
        roller.runVoltage(0);
    }

    /**
     * Drives the pivot back and forth between the open and closed hard stops while
     * SHOOTING,
     * flipping on a fixed timing interval (kOscillationIntervalSecs) rather than
     * waiting for a
     * stall - a quick predictable shake agitates the fuel better than a full
     * hard-stop swing.
     *
     * Deliberately NOT using ConditionalAction here - re-arming it on every flip
     * still stopped
     * toggling after the first cycle. Instead this just records a start time once,
     * and update()
     * recomputes which side we should be on from elapsed wall-clock time every
     * loop, with no
     * registered event to lose track of.
     */
    private void oscillating() {
        clearConditionalActions();
        oscillationTarget = OscillationTarget.OPEN;
        io.runVoltsPivot(IntakeConstants.kPivotOpenVolts);
        oscillationStartTime = Timer.getFPGATimestamp();
    }

    private void updateOscillation() {
        double elapsed = Timer.getFPGATimestamp() - oscillationStartTime;
        long cycleIndex = (long) (elapsed / IntakeConstants.kOscillationIntervalSecs);
        OscillationTarget desired = (cycleIndex % 2 == 0) ? OscillationTarget.OPEN : OscillationTarget.CLOSED;

        if (desired != oscillationTarget) {
            oscillationTarget = desired;
            io.runVoltsPivot(desired == OscillationTarget.OPEN
                    ? IntakeConstants.kPivotOpenVolts
                    : IntakeConstants.kPivotClosedVolts);
        }
    }

    @Override
    public void update() {
        if (Superstate.getInstance().isCurrentWanted(RobotState.SHOOTING)) {
            updateOscillation();
        }

        if (TunableManager.checkChanged(this)) {
            io.runVoltsPivot(tunablePivotVolts);
            roller.runVoltage(tunableRollerVolts);
        }
    }

    @Override
    public boolean isReady() {
        if (Superstate.getInstance().isCurrentWanted(RobotState.INTAKING)
                || Superstate.getInstance().isCurrentWanted(RobotState.HOME))
            return isPivotStalled(pivotStallDebouncer);
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