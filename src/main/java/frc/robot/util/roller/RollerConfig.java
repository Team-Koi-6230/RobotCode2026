package frc.robot.util.roller;

public class RollerConfig {
    public enum RollerMotor {
        NEO,
        VORTEX
    }

    public RollerConfig() {
    }

    public RollerMotor motor = RollerMotor.NEO;
    public String name = "Roller";
    public int motorId = 0;
    public double gearRatio = 1;
    public double MOI = 0.012;
    public int smartCurrentLimit = 20;
}
