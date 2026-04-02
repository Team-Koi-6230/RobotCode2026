package frc.robot.subsystems.intake.io;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeIOSim implements IntakeIO {
    private final SingleJointedArmSim pivotSim;

    @AutoLogOutput
    public final LoggedMechanism2d _pivotMech;
    private final LoggedMechanismRoot2d _pivotRoot;
    private final LoggedMechanismLigament2d _pivotViz;

    private final ProfiledPIDController pivotController;
    private final ArmFeedforward pivotFeedforward;

    private boolean _closedLoopPivot = false;

    private Rotation2d _targetAngle = IntakeConstants.kMaxAngle;
    private double _pivotVoltage = 0;

    public IntakeIOSim() {

        pivotSim = new SingleJointedArmSim(
                DCMotor.getNEO(1),
                IntakeConstants.kPivotMotorToPivotGearRatio,
                IntakeConstants.kMOIpivot,
                IntakeConstants.kLengthPivot,
                IntakeConstants.kMinAngle.getRadians(),
                IntakeConstants.kMaxAngle.getRadians(),
                true,
                IntakeConstants.kMaxAngle.getRadians(),
                0.001, 0.001);

        _pivotMech = new LoggedMechanism2d(2.0, 2.0);
        _pivotRoot = _pivotMech.getRoot("Pivot", 1.0, 1.0);
        _pivotViz = _pivotRoot.append(
                new LoggedMechanismLigament2d("IntakePivot", IntakeConstants.kLengthPivot, 0));

        var constraints = new TrapezoidProfile.Constraints(
                IntakeConstants.kMaxVelocityRadPerSec,
                IntakeConstants.kMaxAccelRadPerSecSquared);

        pivotController = new ProfiledPIDController(
                IntakeConstants.kPsim,
                IntakeConstants.kIsim,
                IntakeConstants.kDsim,
                constraints);

        pivotFeedforward = new ArmFeedforward(
                IntakeConstants.kSsim,
                IntakeConstants.kGsim,
                IntakeConstants.kVsim,
                IntakeConstants.kAsim);
    }

    @Override
    public void updateInputs(IntakeIOInputsAutoLogged inputs) {
        if (DriverStation.isDisabled()) {
            pivotSim.setInputVoltage(0);
            _pivotVoltage = 0;
        } else if (_closedLoopPivot) {
            handlePivotClosedLoop();
        } else {
            pivotSim.setInputVoltage(_pivotVoltage);
        }

        pivotSim.update(Constants.loopPeriodcSecs);
        setInputs(inputs);
    }

    private void handlePivotClosedLoop() {
        var PIDoutput = pivotController.calculate(pivotSim.getAngleRads());
        var setpoint = pivotController.getSetpoint();
        var ffOutput = pivotFeedforward.calculate(setpoint.position, setpoint.velocity);

        _pivotVoltage = MathUtil.clamp(PIDoutput + ffOutput, -12, 12);
        pivotSim.setInputVoltage(_pivotVoltage);
    }

    @Override
    public void setTargetAngle(Rotation2d targetAngle) {
        _closedLoopPivot = true;
        if (_targetAngle.getDegrees() == targetAngle.getDegrees())
            return;

        pivotController.setGoal(targetAngle.getRadians());
        _targetAngle = targetAngle;
    }

    @Override
    public void runVoltsPivot(double volts) {
        _closedLoopPivot = false;
        _pivotVoltage = MathUtil.clamp(volts, -12.0, 12.0);

        pivotSim.setInputVoltage(-_pivotVoltage);
    }

    private void setInputs(IntakeIOInputsAutoLogged inputs) {
        inputs.pivotTargetAngle = _targetAngle.getDegrees();
        inputs.pivotAngleError = Math.abs(_targetAngle.getDegrees() - getPivotAngle().getDegrees());
        inputs.absolutePivotAngleRad = getPivotAngle().getRadians();
        inputs.relativePivotAngleRad = getPivotAngle().getRadians();
        inputs.absolutePivotAngleDeg = getPivotAngle().getDegrees();
        inputs.relativePivotAngleDeg = getPivotAngle().getDegrees();
        inputs.pivotClosedLoop = _closedLoopPivot;
        inputs.pivotAppliedVoltage = _pivotVoltage;
        inputs.pivotCurrent = new double[] { pivotSim.getCurrentDrawAmps() };

        _pivotViz.setAngle(Math.toDegrees(pivotSim.getAngleRads()));
    }

    private Rotation2d getPivotAngle() {
        return Rotation2d.fromRadians(pivotSim.getAngleRads());
    }

    @Override
    public void stop() {
        pivotSim.setInputVoltage(0);
    }

    @Override
    public void setPIDF(double kP, double kI, double kD, double kS, double kG, double kV, double kA,
            double kMaxVelocityRadPerSec, double kMaxAccelRadPerSecSquared) {
        pivotController.setPID(kP, kI, kD);
        pivotFeedforward.setKs(kS);
        pivotFeedforward.setKg(kG);
        pivotFeedforward.setKv(kV);
        pivotFeedforward.setKa(kA);
        pivotController.setConstraints(new Constraints(kMaxVelocityRadPerSec, kMaxAccelRadPerSecSquared));
    }
}