package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import team6230.koiupstream.subsystems.UpstreamSubsystem;
import team6230.koiupstream.superstates.Superstate;

public class LED extends UpstreamSubsystem<RobotState, LEDIO, LEDIOInputsAutoLogged> {
    private static final LEDIO ioInstance = Constants.currentMode == Mode.REAL ? new LEDIOreal() : new LEDIO() {
    };

    private LEDPattern activePattern = LEDPattern.solid(Color.kBlack);
    private LEDPattern nextPattern = activePattern;

    public LED() {
        super("LED", new LEDIOInputsAutoLogged());
        addSuperstateBehaviour(RobotState.IDLE, () -> nextPattern = LEDConstants.kIdleLedPattern);
        addDefaultSuperstateBehaviour(() -> nextPattern = LEDConstants.kIdleLedPattern);
        addSuperstateBehaviour(RobotState.UNJAM, () -> nextPattern = LEDConstants.kUnjamLedPattern);
        addSuperstateBehaviour(RobotState.INTAKING, () -> nextPattern = LEDConstants.kIntakeLedPattern);
        addSuperstateBehaviour(RobotState.PREPARING_SHOOTER_AND_INTAKING,
                () -> nextPattern = LEDConstants.kIntakeLedPattern);
        addSuperstateBehaviour(RobotState.SHOOTING_AND_INTAKING, () -> nextPattern = LEDConstants.kIntakeLedPattern);
        addSuperstateBehaviour(RobotState.PREPARING_SHOOTER,
                () -> nextPattern = LEDConstants.kPreparingShooterLedPattern);
        addSuperstateBehaviour(RobotState.SHOOTING, () -> nextPattern = LEDConstants.kShootingLedPattern);
        addSuperstateBehaviour(RobotState.PRESHOOTING, () -> nextPattern = LEDConstants.kPreparingShooterLedPattern);
    }

    @Override
    public void update() {
        if (!nextPattern.equals(activePattern)) {
            activePattern = nextPattern;
            io.applyPattern(activePattern);
        }
        inputs.activePattern = Superstate.getInstance().getSuperstate().toString();
    }

    @Override
    public boolean isReady() {
        return true;
    }

    @Override
    protected LEDIO getIO() {
        return ioInstance;
    }
}