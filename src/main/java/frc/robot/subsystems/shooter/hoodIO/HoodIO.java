package frc.robot.subsystems.shooter.hoodIO;

import org.littletonrobotics.junction.AutoLog;

import team6230.koiupstream.io.UpstreamIO;

public interface HoodIO extends UpstreamIO<HoodIOInputsAutoLogged> {
    @AutoLog
    public class HoodIOInputs {
        public double servo1Position = 0.0;
        public double servo2Position = 0.0;
        public double servo1Voltage = 0.0;
        public double servo2Voltage = 0.0;
    }

    public default void updateInputs(HoodIOInputsAutoLogged inputs) {
    }

    public default void setServosPositions(double angle) {
    }
}
