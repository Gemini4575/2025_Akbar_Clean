package frc.robot.commands.driving;

import static frc.robot.Constants.SwerveConstants.MaxMetersPersecond;

import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.model.PathContainer;
import frc.robot.subsystems.drivetrainIOLayers.DrivetrainIO;

// TODO we need to handle alliance colors somewhere

public class DriveToLocation extends Command {

    private static final double TURN_PRECISION = 5 * Math.PI / 180;
    private static final double MAX_SPEED_GLOBAL = 1.0;
    private static final TreeMap<Double, Double> MAX_SPEEDS = new TreeMap<>(
            Map.of(0.0, 0.1, 0.2, 0.2, 0.5, 0.25, 1.5, 1.0));

    private static final double MAX_ANGULAR_SPEED = Math.PI; // radians per second

    private static final double DRIVE_PRECISION = 0.05; // meters

    private final PathContainer pathContainer;
    private final DrivetrainIO driveSubsystem;

    private int segmentIdx = 0;

    private final Field2d targetField = new Field2d();

    public DriveToLocation(DrivetrainIO driveSubsystem, PathContainer pathContainer) {
        this.driveSubsystem = driveSubsystem;
        this.pathContainer = pathContainer;

        initialize();

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        segmentIdx = 0;
        targetField.setRobotPose(pathContainer.getWaypoints().get(0));
        SmartDashboard.putData("[DriveToLocation] Target Pose", targetField);
    }

    // TODO problems we need to smooth out
    // - we should drive faster when we are far, and slower when close
    // - we should try to rotate more smoothly - while driving but ideally not too
    // close to end
    // - rotation in place for short angles is not ideal - we need to detect that we
    // are standing and rotate slower

    @Override
    public void execute() {

        var currentPose = driveSubsystem.getPose();
        Double currentPoseX = currentPose.getX();
        Double currentPoseY = currentPose.getY();
        var targetPose = pathContainer.getWaypoints().get(segmentIdx);
        Double targetPoseX = targetPose.getX();
        Double targetPoseY = targetPose.getY();

        var distanceFromTarget = getDistanceFromTarget();
        double maxSpeed = getMaxSpeed(distanceFromTarget);

        Double xDiff = targetPose.getX() - currentPose.getX();
        Double yDiff = targetPose.getY() - currentPose.getY();
        Double maxDiff = Math.max(Math.abs(xDiff), Math.abs(yDiff));
        Double xSpeed = (xDiff / maxDiff) * maxSpeed;
        Double ySpeed = (yDiff / maxDiff) * maxSpeed;

        Double rotationDiff = distanceFromTarget.getSecond();

        double rotationSpeed = calcAngularSpeed(rotationDiff,
                calcRemainingTime(distanceFromTarget.getFirst(), maxSpeed));

        if (distanceFromTarget.getFirst() < DRIVE_PRECISION) {
            // stop moving if we reach close enough to target
            xSpeed = 0.0;
            ySpeed = 0.0;
        }

        driveSubsystem.log(currentPoseX.toString() + ","
                + currentPoseY.toString() + ","
                + targetPoseX.toString() + ","
                + targetPoseY.toString() + ","
                + xDiff.toString() + ","
                + yDiff.toString() + ",");

        driveSubsystem.drive(xSpeed, ySpeed, rotationSpeed, true);
    }

    private Double getMaxSpeed(Pair<Double, Double> distanceFromTarget) {
        if (locationNeedsToBePrecise()) {
            var speedEntry = MAX_SPEEDS.floorEntry(distanceFromTarget.getFirst());
            return speedEntry == null ? MAX_SPEEDS.firstEntry().getValue() : speedEntry.getValue();
        } else {
            return MAX_SPEED_GLOBAL;
        }
    }

    private boolean locationNeedsToBePrecise() {
        // slow down for last segment
        return segmentIdx == pathContainer.getWaypoints().size() - 1;
    }

    @Override
    public boolean isFinished() {
        var distance = getDistanceFromTarget();
        SmartDashboard.putNumber("DriveToLocation - Distance", distance.getFirst());
        SmartDashboard.putNumber("DriveToLocation - Angular Distance", distance.getSecond());

        if (segmentIdx < pathContainer.getWaypoints().size() - 1) {
            if (distance.getFirst() < DRIVE_PRECISION) {
                segmentIdx++;
                targetField.setRobotPose(pathContainer.getWaypoints().get(segmentIdx));
            }
            return false;
        }
        return distance.getFirst() < DRIVE_PRECISION // Finish when within 10 cm of target
                && Math.abs(distance.getSecond()) < TURN_PRECISION; // within 5 degrees
    }

    private Pair<Double, Double> getDistanceFromTarget() {
        var pose = driveSubsystem.getPose();
        var targetPose = pathContainer.getWaypoints().get(segmentIdx);
        var rawAngularDiff = targetPose.getRotation().getRadians() - pose.getRotation().getRadians();
        var optimizedAngularDiff = optimizeAngle(rawAngularDiff);
        return Pair.of(pose.getTranslation().getDistance(targetPose.getTranslation()),
                optimizedAngularDiff);
    }

    // ensure we turn in the shortest direction
    private double optimizeAngle(double rawAngularDiff) {
        if (rawAngularDiff > Math.PI) {
            return rawAngularDiff - 2.0 * Math.PI;
        }
        if (rawAngularDiff < -Math.PI) {
            return rawAngularDiff + 2.0 * Math.PI;
        }
        return rawAngularDiff;
    }

    // attempt to estimate remaining drive time in seconds
    private double calcRemainingTime(double distance, double maxSpeed) {
        return distance / (MaxMetersPersecond * maxSpeed);
    }

    private double calcAngularSpeed(double angularDiff, double remainingDriveTime) {
        return Math.max(-MAX_ANGULAR_SPEED,
                Math.min(MAX_ANGULAR_SPEED,
                        calcAngularSpeedRaw(angularDiff, remainingDriveTime)));
    }

    private double calcAngularSpeedRaw(double angularDiff, double remainingDriveTime) {
        if (!locationNeedsToBePrecise() && remainingDriveTime < 0.05) {
            return 0.0;
        }
        if (remainingDriveTime > 0.2) {
            return angularDiff / (remainingDriveTime);
        }
        return angularDiff;
    }

}
