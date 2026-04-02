package frc.robot.util.roller.io;

import org.littletonrobotics.junction.AutoLog;

import team6230.koiupstream.io.UpstreamIO;
import team6230.koiupstream.io.UpstreamIO.UpstreamIOInputs;

public interface RollerIO extends UpstreamIO<RollerIOInputsAutoLogged> {
    @AutoLog
    public static class RollerIOInputs extends UpstreamIOInputs {
        public double appliedVoltage = 0.0;
        public double radsPerSec = 0.0;
        public double[] rollerCurrent = { 0.0 };
    }
    

    public default void runVoltage(double volt) {
    };
}
