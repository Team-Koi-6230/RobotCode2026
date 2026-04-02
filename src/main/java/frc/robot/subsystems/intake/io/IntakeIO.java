package frc.robot.subsystems.intake.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import team6230.koiupstream.io.UpstreamIO;
import team6230.koiupstream.io.UpstreamIO.UpstreamIOInputs;

public interface IntakeIO extends UpstreamIO<IntakeIOInputsAutoLogged> {
    @AutoLog
    public static class IntakeIOInputs extends UpstreamIOInputs {
        public double pivotTargetAngle = 0.0;
        public double pivotAngleError = 0.0;
        public double relativePivotAngleRad = 0.0;
        public double absolutePivotAngleRad = 0.0;
        public double relativePivotAngleDeg = 0.0;
        public double absolutePivotAngleDeg = 0.0;
        public double pivotAppliedVoltage = 0.0;
        public double[] pivotCurrent = { 0.0 };
        public boolean pivotClosedLoop = false;
    }

    public default void runVoltsPivot(double volts) {
    }

    public default void setTargetAngle(Rotation2d angle) {
    }

    public default void stop() {
    };

    public default void setPIDF(double kP, double kI, double kD, double kS, double kG, double kV, double kA,
            double kMaxVelocityRadPerSec, double kMaxAccelRadPerSecSquared) {
    };

}