package frc.robot.subsystems.led;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class LEDConstants {
    public static int kLength = cmToPixels(190);

    public static LEDPattern kIdleLedPattern = LEDPattern.solid(new Color("#f100ff"));

    public static LEDPattern kIntakeLedPattern = LEDPattern.solid(new Color("#d9ff00")) 
            .blink(Time.ofBaseUnits(0.1, Seconds), Time.ofBaseUnits(0.04, Seconds));

    public static LEDPattern kPreparingShooterLedPattern = LEDPattern.solid(new Color("#00f2ff"))
            .blink(Time.ofBaseUnits(0.1, Seconds), Time.ofBaseUnits(0.04, Seconds));

    public static LEDPattern kShootingLedPattern = LEDPattern.solid(new Color("#0044ff")) 
            .blink(Time.ofBaseUnits(0.1, Seconds), Time.ofBaseUnits(0.04, Seconds));

    public static LEDPattern kUnjamLedPattern = LEDPattern.solid(new Color("#ff9500"));

    private static int cmToPixels(double lengthCm, int ledsPerMeter) {
        double ledsPerCm = ledsPerMeter / 100.0;
        return (int) Math.round(lengthCm * ledsPerCm);
    }

    private static int cmToPixels(double lengthCm) {
        return cmToPixels(lengthCm, 60);
    }
}