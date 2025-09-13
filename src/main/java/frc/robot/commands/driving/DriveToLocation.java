package frc.robot.commands.driving;

import static frc.robot.Constants.SwerveConstants.MaxMetersPersecond;

import java.util.Map;
import java.util.TreeMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrainIOLayers.DrivetrainIO;

public class DriveToLocation extends Command {

    private static final TreeMap<Double, Double> MAX_SPEEDS = new TreeMap<>(
            Map.of(0.0, 0.1, 0.2, 0.2, 0.5, 0.25, 1.5, 1.0));

    private final Pose2d targetPose;
    private final DrivetrainIO driveSubsystem;
    private final Field2d targetField = new Field2d();

    public DriveToLocation(DrivetrainIO driveSubsystem, Pose2d targetPose) {
        this.driveSubsystem = driveSubsystem;
        this.targetPose = targetPose;

        targetField.setRobotPose(targetPose);
        SmartDashboard.putData("[DriveToLocation] Target Pose", targetField);

        addRequirements(driveSubsystem);
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

        driveSubsystem.log(currentPoseX.toString() + ","
                + currentPoseY.toString() + ","
                + targetPoseX.toString() + ","
                + targetPoseY.toString() + ","
                + xDiff.toString() + ","
                + yDiff.toString() + ",");

        driveSubsystem.drive(xSpeed, ySpeed, rotationSpeed, true);
    }

    private Double getMaxSpeed(Pair<Double, Double> distanceFromTarget) {
        var speedEntry = MAX_SPEEDS.floorEntry(distanceFromTarget.getFirst());
        return speedEntry == null ? MAX_SPEEDS.firstEntry().getValue() : speedEntry.getValue();
    }

    @Override
    public boolean isFinished() {
        var distance = getDistanceFromTarget();
        SmartDashboard.putNumber("DriveToLocation - Distance", distance.getFirst());
        SmartDashboard.putNumber("DriveToLocation - Angular Distance", distance.getSecond());
        return distance.getFirst() < 0.05 // Finish when within 10 cm of target
                && Math.abs(distance.getSecond()) < 5 * Math.PI / 180; // within 5 degrees
    }

    private Pair<Double, Double> getDistanceFromTarget() {
        var pose = driveSubsystem.getPose();
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
        if (remainingDriveTime > 0.5) {
            return angularDiff / (2.0 * remainingDriveTime);
        }
        return Math.signum(angularDiff) * Math.min(Math.abs(angularDiff), Math.PI / 2);
    }

}
