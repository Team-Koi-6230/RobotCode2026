package frc.robot.util.roller;

import frc.robot.Constants;
import frc.robot.util.roller.io.RollerIO;
import frc.robot.util.roller.io.RollerIOSim;
import frc.robot.util.roller.io.RollerIOSpark;

public class Roller {
    public static RollerIO makeRollerIO(RollerConfig config) {
        switch (Constants.currentMode) {
            case REAL:
                return new RollerIOSpark(config);
            case SIM:
                return new RollerIOSim(config);
            case REPLAY:
                return new RollerIO() {
                };
        }
        return new RollerIO() {
        };
    }
}
