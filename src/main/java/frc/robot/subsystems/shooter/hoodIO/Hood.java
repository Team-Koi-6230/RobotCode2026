package frc.robot.subsystems.shooter.hoodIO;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants;

public class Hood {
    public static HoodIO getHoodIO() {
        switch (Constants.currentMode) {
            case REAL:
                return new HoodIOReal();

            case SIM:
                return new HoodIOSim();

            case REPLAY:
                return new HoodIO() {
                };
        }
        return new HoodIO() {
        };
    }

    /*
     * @return returns an array with hood voltages
     */
    public static double[] hoodAngleToVoltages(double angle) {
        angle = MathUtil.clamp(angle, 0, ShooterConstants.Hood.kHoodMaxAngle);
        double primaryVoltage = angle / ShooterConstants.Hood.kHoodMaxAngle;
        return new double[] {
                primaryVoltage,
                1 - primaryVoltage
        };
    }
}
