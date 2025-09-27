package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.model.PathContainer;

public class LocationData {

    public static final double LASER_OFFSET = 0.23;

    // points of interest
    public static final Pose2d REEF_FRONT = new Pose2d(5.721, 4.0259, Rotation2d.fromDegrees(90));
    public static final Pose2d REEF_FRONT_LEFT = new Pose2d(5.124, 2.928, Rotation2d.fromDegrees(30));

    // paths
    public static final PathContainer START_TO_REEF_FRONT = new PathContainer()
            .addWaypoint(REEF_FRONT, LASER_OFFSET);
    public static final PathContainer START_TO_REEF_FRONT_LEFT = new PathContainer()
            .addWaypoint(new Pose2d(
                    5.905,
                    3.018, REEF_FRONT_LEFT.getRotation()))
            .addWaypoint(REEF_FRONT_LEFT, LASER_OFFSET);
}
