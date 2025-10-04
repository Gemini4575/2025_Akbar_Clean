package frc.robot.commands.driving;

import static frc.robot.Constants.SwerveConstants.MaxMetersPersecond;

import java.util.Map;
import java.util.TreeMap;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.model.PathContainer;
import frc.robot.subsystems.drivetrainIOLayers.DrivetrainIO;

public class DriveToLocation extends Command {

    private static final double TURN_PRECISION = 5 * Math.PI / 180;
    private static final double MAX_SPEED_GLOBAL = 1.0;
    private static final double LASER_GUIDED_SPEED = 0.1;
    private static final TreeMap<Double, Double> MAX_SPEEDS = new TreeMap<>(
            Map.of(0.0, 0.1, 0.2, 0.4, 0.5, 0.7, 1.5, 1.0));

    private static final double MAX_ANGULAR_SPEED = Math.PI / 2; // radians per second

    private static final double DRIVE_PRECISION = 0.05; // meters
    private static final double LASER_DRIVING_STUCK_THRESHOLD = 0.5; // meters

    private static final int VISION_DELAY_TOLERANCE = 50000; // milliseconds - dont use for now
    private static final double VISION_CORRECTION = 0.2; // multiplier for time driven without vision vs time with
                                                         // vision

    private final PathContainer pathContainer;
    private final DrivetrainIO driveSubsystem;
    private final LaserCan laserCan;

    private int segmentIdx = 0;
    private Pose2d segmentStartPose;
    private long segmentDriveStartTime = 0;
    private long segmentDriveWithoutVisionStartTime = 0;
    private boolean startedDrivingWithoutVision = false;
    private boolean laserGuidedDrive = false;
    private Pose2d laserGuidedStartingPoint;

    private final Field2d targetField = new Field2d();

    public DriveToLocation(DrivetrainIO driveSubsystem, LaserCan laserCan, PathContainer pathContainer) {
        this.driveSubsystem = driveSubsystem;
        this.laserCan = laserCan;
        this.pathContainer = pathContainer;

        initialize();

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        segmentIdx = 0;
        segmentDriveStartTime = System.currentTimeMillis();
        segmentDriveWithoutVisionStartTime = 0;
        startedDrivingWithoutVision = false;
        segmentStartPose = driveSubsystem.getPose();
        targetField.setRobotPose(pathContainer.getWaypoint(0));
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

        SmartDashboard.putBoolean("[DriveToLocation] laserGuidedDrive", laserGuidedDrive);
        SmartDashboard.putBoolean("[DriveToLocation] startedDrivingWithoutVision", startedDrivingWithoutVision);

        var currentPose = driveSubsystem.getPose();
        Double currentPoseX = currentPose.getX();
        Double currentPoseY = currentPose.getY();
        var targetPose = pathContainer.getWaypoint(segmentIdx);
        Double targetPoseX = targetPose.getX();
        Double targetPoseY = targetPose.getY();

        var distanceFromTarget = getDistanceFromTarget();

        // if we are already using laser distance, or if we are close enough to target
        // to start using it...
        if (laserGuidedDrive || (distanceFromTarget.getFirst() < DRIVE_PRECISION
                && !isDistanceCloseEnough(distanceFromTarget.getFirst()))) {
            // we are close enough to be guided by laser distance
            // TODO we need to parametrize these values but testing for now
            if (!laserGuidedDrive) {
                laserGuidedDrive = true;
                laserGuidedStartingPoint = currentPose;
            }
            driveSubsystem.drive(0, LASER_GUIDED_SPEED, 0, false);
        } else {

            Double xDiff = targetPose.getX() - currentPose.getX();
            Double yDiff = targetPose.getY() - currentPose.getY();

            if (startedDrivingWithoutVision) {
                if (driveSubsystem.visionUpdateDelayMillis() > VISION_DELAY_TOLERANCE) {
                    // keep going straight based on original vector
                    xDiff = targetPose.getX() - segmentStartPose.getX();
                    yDiff = targetPose.getY() - segmentStartPose.getY();
                } else {
                    // vision is back, stop the timer
                    startedDrivingWithoutVision = false;
                    segmentDriveWithoutVisionStartTime = 0;
                }
            }

            if (!startedDrivingWithoutVision && isDistanceCloseEnough(distanceFromTarget.getFirst())
                    && driveSubsystem.visionUpdateDelayMillis() > VISION_DELAY_TOLERANCE) {
                // we reached close enough to target, but vision is lost
                startedDrivingWithoutVision = true;
                segmentDriveWithoutVisionStartTime = System.currentTimeMillis();
                return;
            }

            double maxSpeed = getMaxSpeed(distanceFromTarget);

            Double maxDiff = Math.max(Math.abs(xDiff), Math.abs(yDiff));
            Double xSpeed = (xDiff / maxDiff) * maxSpeed;
            Double ySpeed = (yDiff / maxDiff) * maxSpeed;

            Double rotationDiff = distanceFromTarget.getSecond();

            double rotationSpeed = calcAngularSpeed(rotationDiff,
                    calcRemainingTime(distanceFromTarget.getFirst(), maxSpeed));

            if (isDistanceCloseEnough(distanceFromTarget.getFirst())) {
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
        return segmentIdx == pathContainer.getNumWaypoints() - 1;
    }

    @Override
    public boolean isFinished() {
        var distance = getDistanceFromTarget();
        SmartDashboard.putNumber("DriveToLocation - Distance", distance.getFirst());
        SmartDashboard.putNumber("DriveToLocation - Angular Distance", distance.getSecond());

        if (segmentIdx < pathContainer.getNumWaypoints() - 1) {
            if (isDistanceCloseEnough(distance.getFirst()) && isVisionDriftAcceptable()) {
                segmentIdx++;
                segmentDriveStartTime = System.currentTimeMillis();
                segmentDriveWithoutVisionStartTime = 0;
                startedDrivingWithoutVision = false;
                laserGuidedDrive = false;
                segmentStartPose = driveSubsystem.getPose();
                targetField.setRobotPose(pathContainer.getWaypoint(segmentIdx));
            }
            return false;
        }
        return isDistanceCloseEnough(distance.getFirst())
                && Math.abs(distance.getSecond()) < TURN_PRECISION; // within 5 degrees
    }

    private boolean isVisionDriftAcceptable() {
        var visionDelay = driveSubsystem.visionUpdateDelayMillis();
        if (visionDelay < VISION_DELAY_TOLERANCE) {
            return true;
        }
        return timeDrivenWithoutVision() >= segmentDriveTime() * VISION_CORRECTION;
    }

    private long timeDrivenWithoutVision() {
        return segmentDriveWithoutVisionStartTime == 0 ? 0
                : System.currentTimeMillis() - segmentDriveWithoutVisionStartTime;
    }

    private long segmentDriveTime() {
        return System.currentTimeMillis() - segmentDriveStartTime;
    }

    private boolean isDistanceCloseEnough(double distance) {
        if (laserGuidedDrive || distance < DRIVE_PRECISION) {
            Double laserDistance = pathContainer.getLaserDistance(segmentIdx);
            if (laserDistance == null || laserCan == null) {
                return true;
            }
            // detect if we are stuck and not moving
            double laserDrivenDistance = laserGuidedStartingPoint == null ? 0
                    : laserGuidedStartingPoint.getTranslation()
                            .getDistance(driveSubsystem.getPose().getTranslation());
            return laserDrivenDistance > LASER_DRIVING_STUCK_THRESHOLD
                    || Math.abs(laserDistance - (laserCan.getMeasurement().distance_mm / 1000.0)) < DRIVE_PRECISION;
        }
        return false;
    }

    private Pair<Double, Double> getDistanceFromTarget() {
        var pose = driveSubsystem.getPose();
        var targetPose = pathContainer.getWaypoint(segmentIdx);
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
