package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.model.PathContainer;

public class LocationData {

    public static final double LASER_OFFSET = 0.23;

    // points of interest
    public static final Pose2d REEF_FRONT = new Pose2d(5.721, 4.0259, Rotation2d.fromDegrees(90));
    public static final Pose2d REEF_FRONT_LEFT = new Pose2d(5.124, 2.928, Rotation2d.fromDegrees(30));
    public static final Pose2d REEF_FRONT_RIGHT = new Pose2d(5.139, 5.107, Rotation2d.fromDegrees(150));
    public static final Pose2d REEF_BACK = new Pose2d(3.231, 4.055, Rotation2d.fromDegrees(-90));
    public static final Pose2d REEF_BACK_LEFT = new Pose2d(3.817, 2.943, Rotation2d.fromDegrees(-30));
    public static final Pose2d REEF_BACK_RIGHT = new Pose2d(3.847, 5.122, Rotation2d.fromDegrees(-150));
    public static final Pose2d CORAL_STATION_LEFT = new Pose2d(1.112, 0.945, Rotation2d.fromDegrees(143));
    public static final Pose2d CORAL_STATION_RIGHT = new Pose2d(1.112, 7.070, Rotation2d.fromDegrees(33));

    // paths
    public static final PathContainer START_TO_REEF_FRONT = new PathContainer()
            .addWaypoint(REEF_FRONT, LASER_OFFSET);

    public static final PathContainer START_TO_REEF_FRONT_LEFT = new PathContainer()
            .addWaypoint(new Pose2d(
                    5.705,
                    2.900, REEF_FRONT_LEFT.getRotation()))
            .addWaypoint(REEF_FRONT_LEFT, LASER_OFFSET);

    public static final PathContainer START_TO_REEF_FRONT_RIGHT = new PathContainer()
            .addWaypoint(new Pose2d(
                    5.705,
                    5.200, REEF_FRONT_RIGHT.getRotation()))
            .addWaypoint(REEF_FRONT_RIGHT, LASER_OFFSET);

    public static final PathContainer REEF_FRONT_TO_STATION_LEFT = new PathContainer()
            .addWaypoint(new Pose2d(
                    6.040,
                    2.883, CORAL_STATION_LEFT.getRotation()))
            .addWaypoint(CORAL_STATION_LEFT, LASER_OFFSET);

    public static final PathContainer REEF_FRONT_TO_STATION_RIGHT = new PathContainer()
            .addWaypoint(new Pose2d(
                    6.040,
                    5.272, CORAL_STATION_RIGHT.getRotation()))
            .addWaypoint(CORAL_STATION_RIGHT, LASER_OFFSET);

    public static final PathContainer REEF_LEFT_TO_STATION_LEFT = new PathContainer()
            .addWaypoint(new Pose2d(
                    4.4,
                    2.2, CORAL_STATION_LEFT.getRotation()))
            .addWaypoint(CORAL_STATION_LEFT, LASER_OFFSET);

    public static final PathContainer REEF_RIGHT_TO_STATION_RIGHT = new PathContainer()
            .addWaypoint(new Pose2d(
                    4.4,
                    5.850, CORAL_STATION_RIGHT.getRotation()))
            .addWaypoint(CORAL_STATION_RIGHT, LASER_OFFSET);

    public static final PathContainer STATION_LEFT_TO_REEF_BACK_LEFT = new PathContainer()
            .addWaypoint(REEF_BACK_LEFT, LASER_OFFSET);

    public static final PathContainer STATION_RIGHT_TO_REEF_BACK_RIGHT = new PathContainer()
            .addWaypoint(REEF_BACK_RIGHT, LASER_OFFSET);

    public static final PathContainer STATION_ANY_TO_REEF_BACK_CENTER = new PathContainer()
            .addWaypoint(new Pose2d(
                    2.500,
                    4.000, REEF_BACK.getRotation()))
            .addWaypoint(REEF_BACK, LASER_OFFSET);

    public static final PathContainer REEF_BACK_ANY_TO_STATION_RIGHT = new PathContainer()
            .addWaypoint(new Pose2d(
                    1.700,
                    6.500, CORAL_STATION_RIGHT.getRotation()))
            .addWaypoint(CORAL_STATION_RIGHT, LASER_OFFSET);

    public static final PathContainer REEF_BACK_ANY_TO_STATION_LEFT = new PathContainer()
            .addWaypoint(new Pose2d(
                    1.700,
                    1.500, CORAL_STATION_LEFT.getRotation()))
            .addWaypoint(CORAL_STATION_LEFT, LASER_OFFSET);
}
