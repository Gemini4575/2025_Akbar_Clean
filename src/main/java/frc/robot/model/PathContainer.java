package frc.robot.model;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;

public class PathContainer {

    private final List<Pose2d> waypoints = new ArrayList<>();

    public PathContainer addWaypoint(Pose2d location) {
        waypoints.add(location);
        return this;
    }

    public List<Pose2d> getWaypoints() {
        return waypoints;
    }

}
