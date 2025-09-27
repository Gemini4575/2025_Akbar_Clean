package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class CoordinateConverter {

    private static final double FIELD_SIZE_X = 17.548;
    private static final double FIELD_SIZE_Y = 8.052;

    public static Pose2d convertToAllianceCoordinates(Pose2d originalPose) {
        if (shouldFlipCoordinates()) {
            double newX = FIELD_SIZE_X - originalPose.getX();
            double newY = FIELD_SIZE_Y - originalPose.getY();
            return new Pose2d(newX, newY, originalPose.getRotation().plus(Rotation2d.fromDegrees(180)));
        } else {
            return originalPose;
        }
    }

    public static Rotation2d convertToAllianceRotation(Rotation2d originalRotation) {
        if (shouldFlipCoordinates()) {
            return originalRotation.plus(Rotation2d.fromDegrees(180));
        } else {
            return originalRotation;
        }
    }

    private static boolean shouldFlipCoordinates() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

}
