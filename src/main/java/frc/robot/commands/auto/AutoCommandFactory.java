package frc.robot.commands.auto;

import static frc.robot.LocationData.REEF_FRONT_TO_STATION_LEFT;
import static frc.robot.LocationData.REEF_LEFT_TO_STATION_LEFT;
import static frc.robot.LocationData.START_TO_REEF_FRONT;
import static frc.robot.LocationData.START_TO_REEF_FRONT_LEFT;
import static frc.robot.LocationData.START_TO_REEF_FRONT_RIGHT;
import static frc.robot.LocationData.STATION_ANY_TO_REEF_BACK_CENTER;
import static frc.robot.LocationData.STATION_LEFT_TO_REEF_BACK_LEFT;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.coral.lili.AUTOCoral;
import frc.robot.commands.coral.lili.LIPlaceCoral;
import frc.robot.commands.driving.DriveToLocation;
import frc.robot.model.PathContainer;
import frc.robot.subsystems.LiliCoralSubystem;
import frc.robot.subsystems.drivetrainIOLayers.DrivetrainIO;

public class AutoCommandFactory {

    private static final boolean USE_CORAL = false;

    private final LiliCoralSubystem coralSubystem;
    private final DrivetrainIO drivetrainIO;
    private final LaserCan laserCan;

    public AutoCommandFactory(DrivetrainIO drivetrainIO, LaserCan laserCan, LiliCoralSubystem coralSubystem) {
        this.drivetrainIO = drivetrainIO;
        this.laserCan = laserCan;
        this.coralSubystem = coralSubystem;
    }

    private Command dropOneCenter() {
        return new SequentialCommandGroup(drive(START_TO_REEF_FRONT), placeCoral());
    }

    private Command dropOneLeft() {
        return new SequentialCommandGroup(drive(START_TO_REEF_FRONT_LEFT), placeCoral());
    }

    private Command dropOneRight() {
        return new SequentialCommandGroup(drive(START_TO_REEF_FRONT_RIGHT), placeCoral());
    }

    private Command dropTwoFrontCenterBackLeft() {
        return new SequentialCommandGroup(drive(START_TO_REEF_FRONT), placeCoral(), drive(REEF_FRONT_TO_STATION_LEFT),
                pickupCoral(), drive(STATION_LEFT_TO_REEF_BACK_LEFT), placeCoral());
    }

    private Command dropTwoFrontCenterBackCenterLeftStation() {
        return new SequentialCommandGroup(drive(START_TO_REEF_FRONT), placeCoral(), drive(REEF_FRONT_TO_STATION_LEFT),
                pickupCoral(), drive(STATION_ANY_TO_REEF_BACK_CENTER), placeCoral());
    }

    private Command dropTwoFrontLeftBackLeft() {
        return new SequentialCommandGroup(drive(START_TO_REEF_FRONT_LEFT), placeCoral(),
                drive(REEF_LEFT_TO_STATION_LEFT),
                pickupCoral(), drive(STATION_LEFT_TO_REEF_BACK_LEFT), placeCoral());
    }

    private Command dropTwoFrontLeftBackCenter() {
        return new SequentialCommandGroup(drive(START_TO_REEF_FRONT_LEFT), placeCoral(),
                drive(REEF_LEFT_TO_STATION_LEFT),
                pickupCoral(), drive(STATION_ANY_TO_REEF_BACK_CENTER), placeCoral());
    }

    private Command drive(PathContainer path) {
        return new DriveToLocation(drivetrainIO, laserCan, path);
    }

    private Command placeCoral() {
        return USE_CORAL ? new LIPlaceCoral(coralSubystem) : new WaitCommand(1);
    }

    private Command pickupCoral() {
        return USE_CORAL ? new AUTOCoral(coralSubystem) : new WaitCommand(1);
    }

    public SendableChooser<Command> generateAutoOptions() {
        SendableChooser<Command> chooser = new SendableChooser<>();
        chooser.setDefaultOption("[1] Center", dropOneCenter());
        chooser.addOption("[1] Left", dropOneLeft());
        chooser.addOption("[1] Right", dropOneRight());
        chooser.addOption("[2] Center - Left Station - Left Back", dropTwoFrontCenterBackLeft());
        chooser.addOption("[2] Center - Left Station - Center Back", dropTwoFrontCenterBackCenterLeftStation());
        chooser.addOption("[2] Left - Left Station - Left Back", dropTwoFrontLeftBackLeft());
        chooser.addOption("[2] Left - Left Station - Center Back", dropTwoFrontLeftBackCenter());
        return chooser;
    }

}
