package frc.robot.commands.driving;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrainIOLayers.DrivetrainIO;

public class DriveToLocation extends Command {

    private static final double MAX_SPEED = 0.2;

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
        double xDiff = targetPose.getX() - currentPose.getX();
        double yDiff = targetPose.getY() - currentPose.getY();
        double maxDiff = Math.max(Math.abs(xDiff), Math.abs(yDiff));
        double xSpeed = xDiff / maxDiff * MAX_SPEED;
        double ySpeed = yDiff / maxDiff * MAX_SPEED;
        driveSubsystem.drive(ySpeed, xSpeed, 0, true);
    }

    @Override
    public boolean isFinished() {
        return getDistanceFromTarget() < 0.15; // Finish when within 10 cm of target
    }

    private double getDistanceFromTarget() {
        return driveSubsystem.getPose().getTranslation().getDistance(targetPose.getTranslation());
    }

}
