package frc.robot.utils;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.WantedState;

public class LedSubsystem extends SubsystemBase {
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    private double endTime = Double.POSITIVE_INFINITY;

    private WantedState previousWantedState = WantedState.IDLE;
    private WantedState wantedState = WantedState.IDLE;

    private LEDPattern currentPattern = Constants.LEDconstants.kIdleLED;

    public LedSubsystem() {
        led = new AddressableLED(Constants.LEDconstants.kLedPort);
        buffer = new AddressableLEDBuffer(Constants.LEDconstants.kLedCount);

        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();

        if (wantedState != previousWantedState) {
            LEDPattern base = getStatePattern();
            if (base != null) {
                currentPattern = base.blink(
                        Seconds.of(Constants.LEDconstants.kBlinkTime / 4)); // Blink twice during the total blink window
                endTime = now + Constants.LEDconstants.kBlinkTime;
            }
        }

        if (now >= endTime) {
            currentPattern = Constants.LEDconstants.kIdleLED;
        }

        currentPattern.applyTo(buffer);
        led.setData(buffer);

        previousWantedState = wantedState;
    }

    private LEDPattern getStatePattern() {
        if (Superstructure.getInstance().isSuperstateMode()) {
            switch (wantedState) {
                case HOME:
                    return Constants.LEDconstants.kIdleLED;
                case PREPARING_SHOOTER:
                    return Constants.LEDconstants.kPrepLED;
                case SHOOTING:
                    return Constants.LEDconstants.kShootLED;
                case INTAKING:
                    return Constants.LEDconstants.kIntakeLED;
                case L1_CLIMB:
                case L3_CLIMB:
                    return Constants.LEDconstants.kClimbsLED;
                default:
                    break;
            }
        }
        return Constants.LEDconstants.kIdleLED;
    }
}
