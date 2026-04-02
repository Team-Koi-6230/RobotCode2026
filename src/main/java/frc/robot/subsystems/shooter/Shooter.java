package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.ballistics.BallisticsCalculator;
import frc.robot.subsystems.shooter.flywheelIO.ShooterIO;
import frc.robot.subsystems.shooter.flywheelIO.ShooterIOInputsAutoLogged;
import frc.robot.subsystems.shooter.flywheelIO.ShooterIORev;
import frc.robot.subsystems.shooter.flywheelIO.ShooterIOSim;
import frc.robot.subsystems.shooter.hoodIO.Hood;
import frc.robot.subsystems.shooter.hoodIO.HoodIO;
import frc.robot.subsystems.shooter.hoodIO.HoodIOInputsAutoLogged;
import frc.robot.util.roller.Roller;
import frc.robot.util.roller.RollerConfig;
import frc.robot.util.roller.io.RollerIO;
import frc.robot.util.roller.io.RollerIOInputsAutoLogged;
import team6230.koiupstream.subsystems.ExtraIO;
import team6230.koiupstream.subsystems.UpstreamSubsystem;
import team6230.koiupstream.superstates.Superstate;
import team6230.koiupstream.tunable.Tunable;
import team6230.koiupstream.tunable.TunableManager;

public class Shooter extends UpstreamSubsystem<RobotState, ShooterIO, ShooterIOInputsAutoLogged> {
    // #region IOs
    private HoodIO hoodIO = Hood.getHoodIO();
    private HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();

    private RollerIO rollerIO;
    private RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();
    // #endregion

    private boolean isShooting = false;

    private BallisticsCalculator ballisticsCalculator = Robot.ballisticsCalculator;

    @Tunable
    private double _tunedSurfaceSpeed = 0;
    @Tunable
    private double _tunedHoodAngle = 0;

    @Tunable
    private double kP = Robot.isReal() ? ShooterConstants.Flywheel.kP : ShooterConstants.Flywheel.kPSim;

    @Tunable
    private double kI = Robot.isReal() ? ShooterConstants.Flywheel.kI : ShooterConstants.Flywheel.kISim;

    @Tunable
    private double kD = Robot.isReal() ? ShooterConstants.Flywheel.kD : ShooterConstants.Flywheel.kDSim;

    @Tunable
    private double kS = Robot.isReal() ? ShooterConstants.Flywheel.kS : ShooterConstants.Flywheel.kSSim;

    @Tunable
    private double kV = Robot.isReal() ? ShooterConstants.Flywheel.kV : ShooterConstants.Flywheel.kVSim;

    @Tunable
    private double kA = Robot.isReal() ? ShooterConstants.Flywheel.kA : ShooterConstants.Flywheel.kASim;

    public Shooter() {
        super("Shooter", new ShooterIOInputsAutoLogged());

        RollerConfig r_config = new RollerConfig();
        r_config.motorId = RobotMap.CanBus.kFeederRollerID;
        r_config.gearRatio = ShooterConstants.Roller.kRollerGearRatio;
        r_config.name = "ShooterFeeder";
        r_config.smartCurrentLimit = ShooterConstants.Roller.kRollerSmartCurrentLimit;
        r_config.motor = ShooterConstants.Roller.kRollerMotor;

        rollerIO = Roller.makeRollerIO(r_config);

        addAnExtraIO(new ExtraIO(hoodIO, hoodInputs, "ShooterHood"));
        addAnExtraIO(new ExtraIO(rollerIO, rollerInputs, r_config.name));
        hoodIO.setServosPositions(ShooterConstants.Hood.kNonShootingAngle);

        addDefaultSuperstateBehaviour(this::stopAll);
        addSuperstateBehaviour(RobotState.IDLE, this::stopAll);
        addSuperstateBehaviour(RobotState.UNJAM, this::unjam);
        addSuperstateBehaviour(RobotState.PREPARING_SHOOTER, this::prepareShooter);
        addSuperstateBehaviour(RobotState.PREPARING_SHOOTER_AND_INTAKING, this::prepareShooter);
        addSuperstateBehaviour(RobotState.SHOOTING, this::shooting);
        addSuperstateBehaviour(RobotState.SHOOTING_AND_INTAKING, this::shooting);
    }

    @Override
    public void update() {
        if (TunableManager.checkChanged(this)) {
            if (DriverStation.isTest()) {
                io.setPIDF(kP, kI, kD, kS, kV, kA);
                io.runRPM(BallisticsCalculator.convertSurfaceVelocityToRotationPerMinute(_tunedSurfaceSpeed));
                hoodIO.setServosPositions(_tunedHoodAngle);
            }
        }

        if (!isShooting || DriverStation.isTest())
            return;

        var flywheelSetpoint = ballisticsCalculator.getFlywheelSetpoint();
        var hoodSetpoint = ballisticsCalculator.getHoodSetpoint();

        if (inputs.targetRPM != flywheelSetpoint)
            io.runRPM(flywheelSetpoint);
        if (hoodInputs.servo1Position != hoodSetpoint)
            hoodIO.setServosPositions(hoodSetpoint);
    }

    @Override
    public boolean isReady() {
        if (Superstate.getInstance().isCurrentWanted(RobotState.SHOOTING) ||
                Superstate.getInstance().isCurrentWanted(RobotState.SHOOTING_AND_INTAKING) ||
                Superstate.getInstance().isCurrentWanted(RobotState.PREPARING_SHOOTER) ||
                Superstate.getInstance().isCurrentWanted(RobotState.PREPARING_SHOOTER_AND_INTAKING)) {
            return Math.abs(inputs.currentRPM - inputs.currentRPM) < ShooterConstants.Flywheel.kRpmErrorTolerance;
        }
        return true;
    }

    private void stopAll() {
        io.stop();
        rollerIO.runVoltage(0);
        hoodIO.setServosPositions(ShooterConstants.Hood.kNonShootingAngle);
        isShooting = false;
    }

    private void unjam() {
        io.runVolts(ShooterConstants.Flywheel.kUnjamVolts);
        rollerIO.runVoltage(ShooterConstants.Roller.kUnjamVolts);
        hoodIO.setServosPositions(ShooterConstants.Hood.kNonShootingAngle);
        isShooting = false;
    }

    private void prepareShooter() {
        isShooting = true;
        rollerIO.runVoltage(0);
    }

    private void shooting() {
        isShooting = true;
        rollerIO.runVoltage(ShooterConstants.Roller.kFeedVolts);
    }

    @Override
    protected ShooterIO getIO() {
        switch (Constants.currentMode) {
            case REAL:
                return new ShooterIORev();
            case SIM:
                return new ShooterIOSim();
            case REPLAY:
                return new ShooterIO() {
                };
        }
        return new ShooterIO() {
        };
    }

}
