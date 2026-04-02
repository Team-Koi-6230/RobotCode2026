package frc.robot.subsystems.shooter.flywheelIO;

import org.littletonrobotics.junction.AutoLog;

import team6230.koiupstream.io.UpstreamIO;
import team6230.koiupstream.io.UpstreamIO.UpstreamIOInputs;

public interface ShooterIO extends UpstreamIO<ShooterIOInputsAutoLogged> {
    @AutoLog
    public class ShooterIOInputs extends UpstreamIOInputs {
        public double appliedVoltage = 0.0;
        public double[] current = new double[] { 0.0, 0.0 };
        public double targetRPM = 0.0;
        public double currentRPM = 0.0;
        public double radsPerSec = 0.0;
    }

    public default void updateInputs(ShooterIOInputsAutoLogged inputs) {
    }

    public default void runVolts(double volts) {
    }

    public default void stop() {
    }

    public default void runRPM(double RPM) {
    }

    public default void setPIDF(double kP, double kI, double kD, double kS, double kV, double kA) {
    }
}