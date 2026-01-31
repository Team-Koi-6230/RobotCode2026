package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.subsystems.Superstructure.WantedState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
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

    private HoodState state = HoodState.AT_STARTING_POS;
    private WantedState currentWantedState = WantedState.IDLE;

    public HoodSubsystem() {
        servoRight = new Servo(Constants.HoodConstants.kServoRightID);
        servoLeft = new Servo(Constants.HoodConstants.kServoLeftID);

        // Start at home
        setAngle(Constants.HoodConstants.kStartingPos);
    }

    public void setAngle(double degrees) {
        degrees = MathUtil.clamp(
                degrees,
                Constants.HoodConstants.kMinDeg,
                Constants.HoodConstants.kMaxDeg
        );

        if (Double.compare(degrees, targetAngle) == 0) {
            return;
        }

        double normalized =
                (degrees - Constants.HoodConstants.kMinDeg) /
                (Constants.HoodConstants.kMaxDeg - Constants.HoodConstants.kMinDeg);

        double leftPwm =
                Constants.HoodConstants.kServoMin +
                normalized * (Constants.HoodConstants.kServoMax - Constants.HoodConstants.kServoMin);

        double rightPwm =
                Constants.HoodConstants.kServoMin +
                (1.0 - normalized) * (Constants.HoodConstants.kServoMax - Constants.HoodConstants.kServoMin);

        servoLeft.set(leftPwm);
        servoRight.set(rightPwm);

        targetAngle = degrees;
        lastSetTime = Timer.getFPGATimestamp();
        state = HoodState.MOVING;
    }

    /** Command wrapper */
    public Command setHoodAngleCommand(double angle) {
        return runOnce(() -> setAngle(angle));
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
        if (Vision.getInstance().isInAllianceZone()) {
            setAngle(Constants.HoodConstants.kAllianceAngle);
        } else {
            setAngle(
                Superstructure.getInstance()
                    .getShooterParameters()
                    .hoodAngle()
            );
        }
    }

    public void setWantedState(WantedState wantedState) {
        this.currentWantedState = wantedState;
    }

    @Override
    public void periodic() {
        if (state == HoodState.MOVING) {
            if (Timer.getFPGATimestamp() - lastSetTime
                    >= Constants.HoodConstants.kServoDelay) {

                state = targetAngle == Constants.HoodConstants.kStartingPos
                        ? HoodState.AT_STARTING_POS
                        : HoodState.AT_TARGET;
            }
        }
        switch (currentWantedState) {
            case IDLE:
            case HOME:
            case INTAKING:
            case L1_CLIMB:
            case L3_CLIMB:
                resetPosition();
                break;

            case PREPARING_SHOOTER:
            case SHOOTING:
                prepareHood();
                break;
        }
    }
}
