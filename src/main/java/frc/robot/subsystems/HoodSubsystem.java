package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure.WantedState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {

    public enum HoodState {
        AT_TARGET,
        AT_STARTING_POS,
        MOVING
    }

    private final Servo servoRight, servoLeft;

    private double targetAngle = Double.NaN;
    private double lastSetTime = 0.0;

    private double debugAngle = 0;

    private HoodState state = HoodState.AT_STARTING_POS;
    private WantedState currentWantedState = WantedState.IDLE;

    public HoodSubsystem() {
        servoRight = new Servo(Constants.HoodConstants.kServoRightID);
        servoLeft = new Servo(Constants.HoodConstants.kServoLeftID);

        // Start at home
        setAngle(Constants.HoodConstants.kStartingPos);

        SmartDashboard.putNumber("Hood/debug Angle", debugAngle);
    }

    public void setAngle(double degrees) {
        double clamped = MathUtil.clamp(
                degrees,
                Constants.HoodConstants.kMinDeg,
                Constants.HoodConstants.kMaxDeg);  

        this.targetAngle = clamped;

        double normalized = (targetAngle - Constants.HoodConstants.kMinDeg) /
                (Constants.HoodConstants.kMaxDeg - Constants.HoodConstants.kMinDeg);

        System.out.println(normalized + " | " + (1.0 - normalized));

        servoLeft.set(normalized);
        servoRight.set(1.0 - normalized);

        SmartDashboard.putNumber("hood/left", normalized);
        SmartDashboard.putNumber("hood/right", 1.0 - normalized);

        lastSetTime = Timer.getFPGATimestamp();
        state = HoodState.MOVING;
    }

    /** Command wrapper */
    public Command setHoodAngleCommand(double angle) {
        return runOnce(() -> setAngle(angle));
    }

    public Command joystickHoodControl(DoubleSupplier axis) {
        return run(() -> {
            double input = MathUtil.applyDeadband(axis.getAsDouble(), 0.05);
            input = Math.copySign(input * input, input);

            double min = Constants.HoodConstants.kMinDeg;
            double max = Constants.HoodConstants.kMaxDeg;
            double mid = (min + max) / 2.0;

            double rangeUp = max - mid;
            double rangeDown = mid - min;

            double targetDeg;
            if (input >= 0) {
                targetDeg = mid + input * rangeUp;
            } else {
                targetDeg = mid + input * rangeDown;
            }

            setAngle(targetDeg);
        });
    }

    public HoodState getState() {
        return state;
    }

    public boolean isReady() {
        return state == HoodState.AT_TARGET;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    private void resetPosition() {
        setAngle(Constants.HoodConstants.kStartingPos);
    }

    private void prepareHood() {
        if (!Vision.getInstance().isInAllianceZone()) {
            setAngle(Constants.HoodConstants.kAllianceAngle);
        } else {
            setAngle(Vision.getInstance().getHoodAngle());
        }
    }

    public void setWantedState(WantedState wantedState) {
        this.currentWantedState = wantedState;
    }

    @Override
    public void periodic() {
        tuning();

        if (state == HoodState.MOVING) {
            if (Timer.getFPGATimestamp() - lastSetTime >= Constants.HoodConstants.kServoDelay) {
                state = (Math.abs(targetAngle - Constants.HoodConstants.kStartingPos) < 0.1)
                        ? HoodState.AT_STARTING_POS
                        : HoodState.AT_TARGET;
            }
        }

        if (Superstructure.getInstance().isManualMode())
            return;

        switch (currentWantedState) {
            case IDLE:
            case HOME:
            case INTAKING:
            case UNJAM:
            case L1_CLOSE_AUTON:
            case L1_CLOSE_TELEOP:
            case L1_EXTEND_AUTON:
            case L1_EXTEND_TELEOP:
                if (targetAngle != Constants.HoodConstants.kStartingPos) {
                    resetPosition();
                }
                break;
            case PREPARING_SHOOTER_AND_INTAKING:
            case SHOOTING_AND_INTAKING:
            case PREPARING_SHOOTER:
            case SHOOTING:
                prepareHood();
                break;
        }
    }

    private void tuning() {
        if (!DriverStation.isTest())
            return;
        debugAngle = SmartDashboard.getNumber("Hood/debug Angle", 0);
        if (debugAngle != 0) {
            setAngle(debugAngle);
        }
    }
}
