package frc.robot.subsystems.shooter.flywheelIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants;

public class ShooterIOSim implements ShooterIO {
    private final DCMotor gearbox;
    private final FlywheelSim sim;

    private final PIDController controller;
    private final SimpleMotorFeedforward ffController;

    private boolean isClosedLoop = false;
    private double _targetRPM = 0;
    private double appliedVolts = 0.0;

    public ShooterIOSim() {
        gearbox = DCMotor.getNeoVortex(2);

        LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(
                gearbox,
                0.004,
                1.0);

        sim = new FlywheelSim(plant, gearbox, 0.0001);

        controller = new PIDController(ShooterConstants.Flywheel.kPSim, ShooterConstants.Flywheel.kISim,
                ShooterConstants.Flywheel.kDSim);
        ffController = new SimpleMotorFeedforward(ShooterConstants.Flywheel.kSSim, ShooterConstants.Flywheel.kVSim,
                ShooterConstants.Flywheel.kASim);
    }

    @Override
    public void updateInputs(ShooterIOInputsAutoLogged inputs) {
        if (isClosedLoop) {
            double currentRadsPerSec = sim.getAngularVelocityRadPerSec();
            double targetRadsPerSec = Units.rotationsPerMinuteToRadiansPerSecond(_targetRPM);

            double pidVolts = controller.calculate(currentRadsPerSec, targetRadsPerSec);
            double ffVolts = ffController.calculate(targetRadsPerSec);

            appliedVolts = MathUtil.clamp(pidVolts + ffVolts, -12.0, 12.0);
        }

        sim.setInputVoltage(appliedVolts);
        sim.update(Constants.loopPeriodcSecs);

        inputs.appliedVoltage = appliedVolts;

        double currentPerMotor = sim.getCurrentDrawAmps() / 2.0;
        inputs.current = new double[] { currentPerMotor, currentPerMotor };

        inputs.targetRPM = _targetRPM;
        inputs.radsPerSec = sim.getAngularVelocityRadPerSec();
        inputs.currentRPM = sim.getAngularVelocityRPM();
    }

    @Override
    public void runVolts(double volts) {
        isClosedLoop = false;
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }

    @Override
    public void stop() {
        isClosedLoop = false;
        appliedVolts = 0.0;
    }

    @Override
    public void runRPM(double rpm) {
        isClosedLoop = true;
        _targetRPM = rpm;
    }

    @Override
    public void setPIDF(double kP, double kI, double kD, double kS, double kV, double kA) {
        controller.setPID(kP, kI, kD);
        ffController.setKs(kS);
        ffController.setKv(kV);
        ffController.setKa(kA);
    }
}