package frc.robot.util.roller.io;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.util.roller.RollerConfig;
import frc.robot.util.roller.RollerConfig.RollerMotor;

public class RollerIOSim implements RollerIO {
    private double appliedVoltage = 0;
    private final DCMotorSim sim;

    public RollerIOSim(RollerConfig config) {

        var motor = config.motor == RollerMotor.NEO ? DCMotor.getNEO(1) : DCMotor.getNeoVortex(1);
        var rollerMotor = motor.withReduction(IntakeConstants.kRollerGearRatio);
        sim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(rollerMotor, config.MOI, 0.0001), rollerMotor);
    }

    @Override
    public void updateInputs(RollerIOInputsAutoLogged inputs) {
        inputs.appliedVoltage = appliedVoltage;
        inputs.radsPerSec = sim.getAngularVelocityRadPerSec();
        inputs.rollerCurrent = new double[] { sim.getCurrentDrawAmps() };
    }

    @Override
    public void runVoltage(double volt) {
        appliedVoltage = MathUtil.clamp(volt, -12, 12);
        sim.setInputVoltage(appliedVoltage);
    }
}