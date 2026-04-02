package frc.robot.subsystems.shooter.ballistics;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.LinesHorizontal;
import frc.robot.FieldConstants.LinesVertical;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.AllianceFlipUtil;

public class BallisticsCalculator {
    public double getFlywheelSetpoint() {
        if (Robot.isInAllianceZone())
            return convertSurfaceVelocityToRotationPerMinute(
                    BallisticsParameters.kShotFlywheelSpeedMap.get(getShooterDistanceToHub()));
        return convertSurfaceVelocityToRotationPerMinute(BallisticsParameters.kPassingSpeed);
    }

    public double getHoodSetpoint() {
        if (Robot.isInAllianceZone())
            return BallisticsParameters.kShotHoodAngleMap.get(getShooterDistanceToHub());
        return BallisticsParameters.kPassingAngle;
    }

    private double getShooterDistanceToHub() {
        var shooterTranslation = getShooterPose().getTranslation();
        var hubTranslation = getAllianceHubCenterTranslation();

        return shooterTranslation.getDistance(hubTranslation);
    }

    public static double convertSurfaceVelocityToRotationPerMinute(double surfaceVelocity) {
        return (surfaceVelocity * 60) / (Math.PI * BallisticsParameters.kFlywheelDiameter);
    }

    public static double convertRotationPerMinuteToSurfaceVelocity(double rpm) {
        return (rpm * Math.PI * BallisticsParameters.kFlywheelDiameter) / 60;
    }

    @AutoLogOutput
    public Rotation2d getShootingRobotAngle() {
        return getDriveAngleWithShooterOffset(getRobotShootingTarget());
    }

    @AutoLogOutput
    public Translation2d getRobotShootingTarget() {
        if (Robot.isInAllianceZone())
            return getAllianceHubCenterTranslation();
        return calculatePassingPosition();
    }

    private Translation2d calculatePassingPosition() {
        if (shouldPassLeft()) {
            return calculateLeftLanePassTranslation();
        }
        return calculateRightLanePassTranslation();
    }

    private boolean shouldPassLeft() {
        var robotPosY = AllianceFlipUtil.applyY(RobotContainer.getRobotPose().getY());
        var fieldCenterY = LinesHorizontal.center;

        return robotPosY > fieldCenterY;
    }

    private Translation2d calculateLeftLanePassTranslation() {
        return makeLaneTarget(BallisticsParameters.kLeftLaneMultiplier);
    }

    private Translation2d calculateRightLanePassTranslation() {
        return makeLaneTarget(BallisticsParameters.kRightLaneMultiplier);
    }

    private Translation2d makeLaneTarget(double sideMultiplier) {
        return new Translation2d(
                AllianceFlipUtil.applyX(LinesVertical.center / 4),
                AllianceFlipUtil.applyY(LinesHorizontal.center / 2 * sideMultiplier));
    }

    private Rotation2d getDriveAngleWithShooterOffset(Translation2d targetPose) {
        return targetPose
                .minus(getShooterPose().getTranslation())
                .getAngle();
    }

    private Pose2d getShooterPose() {
        var robotPose = RobotContainer.getRobotPose();
        return robotPose.transformBy(new Transform2d(
                BallisticsParameters.kShooterOffset.getX(),
                BallisticsParameters.kShooterOffset.getY(),
                new Rotation2d()));
    }

    private Translation2d getAllianceHubCenterTranslation() {
        return AllianceFlipUtil.apply(FieldConstants.Hub.innerCenterPoint).toTranslation2d();
    }
}
