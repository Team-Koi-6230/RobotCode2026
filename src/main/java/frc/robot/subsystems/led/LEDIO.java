package frc.robot.subsystems.led;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.LEDPattern;
import team6230.koiupstream.io.UpstreamIO;
import team6230.koiupstream.io.UpstreamIO.UpstreamIOInputs;

public interface LEDIO extends UpstreamIO<LEDIOInputsAutoLogged> {
    @AutoLog
    public class LEDIOInputs extends UpstreamIOInputs {
        public String activePattern = "None";
    }

    @Override
    public default void updateInputs(LEDIOInputsAutoLogged inputs) {
    }

    public default void applyPattern(LEDPattern pattern) {
    }
}
