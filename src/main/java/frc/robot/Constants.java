package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
    public static boolean disableHAL = false;
    public static final double loopPeriodcSecs = 0.02;
    public static final boolean tuningMode = true;

    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }
}
