package frc.robot.subsystems;

import java.io.File;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem.FeederState;
import frc.robot.subsystems.HoodSubsystem.HoodState;
import frc.robot.subsystems.IntakeArmSubsystem.IntakeArmState;
import frc.robot.subsystems.IntakeRollerSubsytem.IntakeRollerState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.SwerveSubsystem.SwerveState;
import frc.robot.utils.GameDataSubsystem;
import frc.robot.utils.RumbleSubsystem;

public class Superstructure extends SubsystemBase {
    private static Superstructure instance;

    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }

    public enum WantedState {
        IDLE,
        HOME,
        INTAKING,
        PREPARING_SHOOTER_AND_INTAKING,
        PREPARING_SHOOTER,
        SHOOTING,
        SHOOTING_AND_INTAKING,
        L1_EXTEND_TELEOP,
        L1_CLOSE_TELEOP,
        L1_EXTEND_AUTON,
        L1_CLOSE_AUTON,
        UNJAM
    }

    public enum CurrentState {
        IDLE,
        HOME,
        INTAKING,
        PREPARING_SHOOTER_AND_INTAKING,
        PREPARING_SHOOTER,
        SHOOTING,
        SHOOTING_AND_INTAKING,
        L1_EXTEND_TELEOP,
        L1_CLOSE_TELEOP,
        L1_EXTEND_AUTON,
        L1_CLOSE_AUTON,
        UNJAM
    }

    public enum RobotControlState {
        SUPERSTATE,
        MANUAL_MODE
    }

    private RobotControlState controlState = RobotControlState.SUPERSTATE;

    private WantedState wantedState = WantedState.IDLE;
    private WantedState previousWantedState = WantedState.IDLE;
    private CurrentState currentState = CurrentState.IDLE;

    private final RumbleSubsystem rumbleSubsystem;
    @SuppressWarnings("unused")
    private final GameDataSubsystem gameDataSubsystem;

    private final ShooterSubsystem shooterSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final IntakeArmSubsystem intakeArmSubsystem;
    private final IntakeRollerSubsytem intakeRollerSubsystem;
    private final HoodSubsystem hoodSubsystem;
    private final SwerveSubsystem drivebase;

    private boolean isSlowMode = false;
    private boolean lastDashboardValue = false;

    private Superstructure() {
        drivebase = new SwerveSubsystem(
                new File(
                        Filesystem.getDeployDirectory(),
                        RobotBase.isSimulation() ? "swerve-sim" : "swerve"));
        rumbleSubsystem = new RumbleSubsystem();
        gameDataSubsystem = new GameDataSubsystem(rumbleSubsystem);

        shooterSubsystem = new ShooterSubsystem();
        shooterSubsystem.setSwerveSubsystem(drivebase);
        feederSubsystem = new FeederSubsystem();
        intakeArmSubsystem = new IntakeArmSubsystem();
        intakeRollerSubsystem = new IntakeRollerSubsytem();
        hoodSubsystem = new HoodSubsystem();

        SmartDashboard.putBoolean("SwerveControl/slow mode", isSlowMode);
    }

    @Override
    public void periodic() {
        boolean dashboardValue = SmartDashboard.getBoolean("SwerveControl/slow mode", isSlowMode);

        if (dashboardValue != lastDashboardValue) {
        isSlowMode = dashboardValue;
    }

        if (wantedState != previousWantedState) {
            onWantedStateChange();
            previousWantedState = wantedState;
        }

        SmartDashboard.putBoolean("SwerveControl/slow mode", isSlowMode);

        updateCurrentState();

        SmartDashboard.putString("superstructure/Wanted", wantedState.name());
        SmartDashboard.putString("superstructure/Current", currentState.name());
        SmartDashboard.putBoolean(
                "superstructure/Superstate Mode", isSuperstateMode());

        lastDashboardValue = SmartDashboard.getBoolean("SwerveControl/slow mode", isSlowMode);
    }

    private void onWantedStateChange() {
        if (isManualMode()) {
            return;
        }

        shooterSubsystem.setWantedState(wantedState);
        feederSubsystem.setWantedState(wantedState);
        hoodSubsystem.setWantedState(wantedState);
        intakeArmSubsystem.setWantedState(wantedState);
        intakeRollerSubsystem.setWantedState(wantedState);
        // climberSubsystem.setWantedState(wantedState);
        drivebase.setWantedState(wantedState);
    }

    private void updateCurrentState() {
        boolean ready = shooterSubsystem.isReady()
                && hoodSubsystem.isReady()
                && intakeArmSubsystem.isReady()
                && intakeRollerSubsystem.isReady()
                && feederSubsystem.isReady()
                && drivebase.isReady();

        if (currentState.ordinal() != wantedState.ordinal() && currentState == CurrentState.IDLE && wantedState != WantedState.IDLE && ready) {
            rumbleSubsystem.changeStateRumble();
        }

        currentState = ready
                ? CurrentState.valueOf(wantedState.name())
                : CurrentState.IDLE;
    }

    public void setControlMode(RobotControlState mode) {
        controlState = mode;
    }

    public void setSuperstateMode() {
        setControlMode(RobotControlState.SUPERSTATE);
    }

    public void setManualMode() {
        setControlMode(RobotControlState.MANUAL_MODE);
    }

    public boolean isSuperstateMode() {
        return controlState == RobotControlState.SUPERSTATE;
    }

    public boolean isManualMode() {
        return controlState == RobotControlState.MANUAL_MODE;
    }

    public void setWantedState(WantedState state) {
        wantedState = state;
    }

    public Command setIDLEstate() {
        return run(() -> setWantedState(WantedState.IDLE));
    }

    public Command setHOMEstate() {
        return run(() -> setWantedState(WantedState.HOME));
    }

    public Command setINTAKINGstate() {
        return run(() -> setWantedState(WantedState.INTAKING));
    }

    public Command setPREPARING_SHOOTERstate() {
        return run(() -> setWantedState(WantedState.PREPARING_SHOOTER));
    }

    public Command setPREPARING_SHOOTER_AND_INTAKINGshooting() {
        return run(
                () -> setWantedState(WantedState.PREPARING_SHOOTER_AND_INTAKING));
    }

    public Command setSHOOTING_AND_INTAKINGshooting() {
        return run(() -> setWantedState(WantedState.SHOOTING_AND_INTAKING));
    }

    public Command setSHOOTINGstate() {
        return run(() -> setWantedState(WantedState.SHOOTING));
    }

    public Command setL1_CLIMBstate() {
        return run(() -> {
            boolean isAuton = DriverStation.isAutonomous();

            boolean isExtended = (currentState == CurrentState.L1_EXTEND_TELEOP ||
                    currentState == CurrentState.L1_EXTEND_AUTON);

            if (isExtended) {
                setWantedState(isAuton ? WantedState.L1_CLOSE_AUTON : WantedState.L1_CLOSE_TELEOP);
            } else {
                setWantedState(isAuton ? WantedState.L1_EXTEND_AUTON : WantedState.L1_EXTEND_TELEOP);
            }
        });
    }

    public Command setUNJAMstate() {
        return run(() -> setWantedState(WantedState.UNJAM));
    }

    public Command autonClimb() {
        return new SequentialCommandGroup(
                runOnce(() -> setWantedState(WantedState.L1_EXTEND_AUTON)),
                new WaitCommand(Constants.PathPlanner.kClimbTimer),
                runOnce(() -> setWantedState(WantedState.L1_CLOSE_AUTON)));
    }

    public Command toggleControlState() {
        return runOnce(() -> {
            if (isSuperstateMode()) {
                setManualMode();
            } else {
                setSuperstateMode();
            }
        });
    }

    public WantedState getWantedState() {
        return wantedState;
    }

    public CurrentState getCurrentState() {
        return currentState;
    }

    public ShooterSubsystem getShooterSubsystem() {
        return shooterSubsystem;
    }

    public FeederSubsystem getFeederSubsystem() {
        return feederSubsystem;
    }

    public IntakeArmSubsystem getIntakeArmSubsystem() {
        return intakeArmSubsystem;
    }

    public IntakeRollerSubsytem getIntakeRollerSubsystem() {
        return intakeRollerSubsystem;
    }

    public HoodSubsystem getHoodSubsystem() {
        return hoodSubsystem;
    }

    public SwerveSubsystem getDrivebase() {
        return drivebase;
    }

    public RumbleSubsystem getRumbleSubsystem() {
        return rumbleSubsystem;
    }

    public ShooterState getShooterState() {
        return shooterSubsystem.getState();
    }

    public FeederState getFeederState() {
        return feederSubsystem.getState();
    }

    public IntakeArmState getIntakeState() {
        return intakeArmSubsystem.getState();
    }

    public IntakeRollerState getIntakeRollerState() {
        return intakeRollerSubsystem.getState();
    }

    public HoodState getHoodState() {
        return hoodSubsystem.getState();
    }

    public SwerveState getSwerveState() {
        return drivebase.getState();
    }

    public double getSwerveHubRelativeRadialSpeed() {
        return drivebase.getHubRelativeVelocity().radialSpeed();
    }

    public double getSwerveHubRelativeStrafeSpeed() {
        return drivebase.getHubRelativeVelocity().radialSpeed();
    }

    public boolean getIsSlowMode() {
        return isSlowMode;
    }

    public void setIsSlowMode(boolean isSlow) {
        this.isSlowMode = isSlow;
    }
}
