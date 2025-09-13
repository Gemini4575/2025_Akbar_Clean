package frc.robot.commands.driving;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrainIOLayers.DrivetrainIO;

public class DriveToLocation extends Command {

    private static final double MAX_SPEED = 0.1;

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

    @Override
    public void execute() {
        var currentPose = driveSubsystem.getPose();
        Double currentPoseX = currentPose.getX();
        Double currentPoseY = currentPose.getY();
        Double targetPoseX = targetPose.getX();
        Double targetPoseY = targetPose.getY();

        Double xDiff = targetPose.getX() - currentPose.getX();
        Double yDiff = targetPose.getY() - currentPose.getY();
        Double maxDiff = Math.max(Math.abs(xDiff), Math.abs(yDiff));
        Double xSpeed = (xDiff / maxDiff) * MAX_SPEED;
        Double ySpeed = (yDiff / maxDiff) * MAX_SPEED;

        Double targetRotation = targetPose.getRotation().getDegrees();
        Double currentRotation = currentPose.getRotation().getDegrees();
        Double rotationDiff = targetRotation - currentRotation;

        rotationDiff = 0.0;

        double rotationSpeed = (rotationDiff / 180) * MAX_SPEED;

        driveSubsystem.log(currentPoseX.toString() + ","
                + currentPoseY.toString() + ","
                + targetPoseX.toString() + ","
                + targetPoseY.toString() + ","
                + xDiff.toString() + ","
                + yDiff.toString() + ",");

        driveSubsystem.drive(xSpeed, ySpeed, 0, true);
    }

    @Override
    public boolean isFinished() {
        var distance = getDistanceFromTarget();
        SmartDashboard.putNumber("DriveToLocation - Distance", distance);
        return distance < 0.2; // Finish when within 10 cm of target
    }

    private double getDistanceFromTarget() {
        return driveSubsystem.getPose().getTranslation().getDistance(targetPose.getTranslation());
    }

}
