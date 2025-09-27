package frc.robot.model;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.util.CoordinateConverter;

public class PathContainer {

    private final List<Pose2d> waypoints = new ArrayList<>();
    private final Map<Integer, Double> laserDistances = new HashMap<>();

    public PathContainer addWaypoint(Pose2d location) {
        waypoints.add(location);
        return this;
    }

    public PathContainer addWaypoint(Pose2d location, double laserDistance) {
        waypoints.add(location);
        laserDistances.put(waypoints.size() - 1, laserDistance);
        return this;
    }

    public Pose2d getWaypoint(int index) {
        return CoordinateConverter.convertToAllianceCoordinates(waypoints.get(index));
    }

    public int getNumWaypoints() {
        return waypoints.size();
    }

    public Double getLaserDistance(int waypointIndex) {
        return laserDistances.get(waypointIndex);
    }

}
