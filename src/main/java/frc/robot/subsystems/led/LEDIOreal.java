package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.RobotMap;

public class LEDIOreal implements LEDIO {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;

    public LEDIOreal() {
        m_led = new AddressableLED(RobotMap.PWM.kLED);
        m_buffer = new AddressableLEDBuffer(LEDConstants.kLength);
        m_led.setLength(LEDConstants.kLength);
        m_led.setData(m_buffer);
        m_led.start();
    }

    @Override
    public void applyPattern(LEDPattern pattern) {
        pattern.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }
}
