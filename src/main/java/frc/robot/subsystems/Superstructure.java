package frc.robot.subsystems;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ClimberSubsystem.ClimberState;
import frc.robot.subsystems.FeederSubsystem.FeederState;
import frc.robot.subsystems.HoodSubsystem.HoodState;
import frc.robot.subsystems.IntakeArmSubsystem.IntakeArmState;
import frc.robot.subsystems.IntakeRollerSubsytem.IntakeRollerState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.SwerveSubsystem.SwerveState;
import frc.robot.utils.GameDataSubsystem;
import frc.robot.utils.RumbleSubsystem;
import frc.robot.utils.ShooterCalc;
import frc.robot.utils.ShooterCalc.ShootingParameters;

public class Superstructure extends SubsystemBase {
    private static Superstructure instance;

    public static Superstructure getInstance() {
        if (instance == null) {
            instance = new Superstructure();
        }
        return instance;
    }

    /* ================= STATES ================= */

    public enum WantedState {
        IDLE,
        HOME,
        INTAKING,
        PREPARING_SHOOTER,
        SHOOTING,
        L1_CLIMB,
        L3_CLIMB
    }

    public enum CurrentState {
        IDLE,
        HOME,
        INTAKING,
        PREPARING_SHOOTER,
        SHOOTING,
        L1_CLIMB,
        L3_CLIMB
    }

    private WantedState wantedState = WantedState.IDLE;
    private WantedState previousWantedState = WantedState.IDLE;
    private CurrentState currentState = CurrentState.IDLE;

    private final Field2d field = new Field2d();

    /* ================= SUBSYSTEMS ================= */

    private final RumbleSubsystem rumbleSubsystem;
    private final GameDataSubsystem gameDataSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final FeederSubsystem feederSubsystem;
    private final IntakeArmSubsystem intakeArmSubsystem;
    private final IntakeRollerSubsytem intakeRollerSubsystem;
    private final ClimberSubsystem climberSubsystem;
    private final HoodSubsystem hoodSubsystem;
    private final SwerveSubsystem drivebase;

    private Superstructure() {
        rumbleSubsystem = new RumbleSubsystem();
        gameDataSubsystem = new GameDataSubsystem(rumbleSubsystem);

        shooterSubsystem = new ShooterSubsystem();
        feederSubsystem = new FeederSubsystem();
        intakeArmSubsystem = new IntakeArmSubsystem();
        intakeRollerSubsystem = new IntakeRollerSubsytem();
        climberSubsystem = new ClimberSubsystem();
        hoodSubsystem = new HoodSubsystem();

        drivebase = new SwerveSubsystem(
                new File(
                        Filesystem.getDeployDirectory(),
                        RobotBase.isSimulation() ? "swerve-sim" : "swerve"));

        SmartDashboard.putData("superstructure/Aiming/Field", field);
    }

    /* ================= PERIODIC ================= */

    @Override
    public void periodic() {
        if (wantedState != previousWantedState) {
            onWantedStateChange();
            previousWantedState = wantedState;
        }

        if (wantedState == WantedState.SHOOTING || wantedState == WantedState.PREPARING_SHOOTER) {
            ShooterCalc.update();
            currentShootingParameters = ShooterCalc.getParameters();
            field.getObject("target").setPose(currentShootingParameters.target());
        }

        updateCurrentState();

        SmartDashboard.putString("superstructure/Wanted superstate", wantedState.name());
        SmartDashboard.putString("superstructure/superstate", currentState.name());
        field.setRobotPose(drivebase.getPose());
    }

    /* ================= STATE FLOW ================= */

    private void onWantedStateChange() {
        shooterSubsystem.setWantedState(wantedState);
        feederSubsystem.setWantedState(wantedState);
        hoodSubsystem.setWantedState(wantedState);
        intakeArmSubsystem.setWantedState(wantedState);
        intakeRollerSubsystem.setWantedState(wantedState);
        climberSubsystem.setWantedState(wantedState);
        drivebase.setWantedState(wantedState);
    }

    private void updateCurrentState() {
        boolean allReady = shooterSubsystem.isReady() &&
                hoodSubsystem.isReady() &&
                intakeArmSubsystem.isReady() &&
                intakeRollerSubsystem.isReady() &&
                feederSubsystem.isReady() &&
                climberSubsystem.isReady() &&
                drivebase.isReady();

        if (allReady) {
            currentState = CurrentState.valueOf(wantedState.name());
        } else {
            currentState = CurrentState.IDLE;
        }
    }

    public void setWantedState(WantedState state) {
        wantedState = state;
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

    public ClimberSubsystem getClimberSubsystem() {
        return climberSubsystem;
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

    public ClimberState getClimberState() {
        return climberSubsystem.getState();
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

    private ShootingParameters currentShootingParameters;

    public ShootingParameters getShooterParameters() {
        return currentShootingParameters;
    }
}
