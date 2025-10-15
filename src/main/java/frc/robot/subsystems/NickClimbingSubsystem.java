package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.epramotor.Motor;
import static frc.robot.Constants.NickClimbingConstanst.*;

public class NickClimbingSubsystem extends SubsystemBase {

    public Motor ClimbingMotor1;
    public Motor ClimbingMotor2;
    private Motor Flipper;

    private boolean isFlipperStopped = false;
    private boolean isClimber1Stopped = false;
    private boolean isClimber2Stopped = false;

    public NickClimbingSubsystem() {
        // Initialization code here
        ClimbingMotor1 = new Motor(ClimbingMotor1_CANID, MotorType.kBrushless);
        ClimbingMotor2 = new Motor(ClimbingMotor2_CANID, MotorType.kBrushless);
        Flipper = new Motor(Flipper_CANID, MotorType.kBrushed);

    }

    private boolean Climb1() {
        // Code to move the elevator
        return ClimbingMotor1.SetIfBoolean(
                Math.abs(ClimbingMotor1.getPosition()) < (ClimbingMotorPoseition),
                ClimbingSpeed);
    }

    private boolean Climb2() {
        // Code to move the elevator
        return ClimbingMotor2.SetIfBoolean(
                Math.abs(ClimbingMotor2.getPosition()) < (ClimbingMotorPoseition),
                ClimbingSpeed);
    }

    public boolean init1() {
        // Code to move the elevator
        if (ClimbingMotor1.getPosition() < Math.round(0)) {
            ClimbingMotor1.set(ClimbingSpeed);
        } else {
            Stop();
            return true;
        }
        return false;
    }

    public boolean init2() {
        // Code to move the elevator
        if (ClimbingMotor2.getPosition() < Math.round(ClimbingMotorPoseition)) {
            ClimbingMotor2.set(ClimbingSpeed);
        } else {
            Stop();
            return true;
        }
        return false;
    }

    public boolean Climb() {
        Climb1();
        Climb2();
        return Climb1() && Climb2();
    }

    public void End() {
        ClimbingMotor1.stop();
        ClimbingMotor2.stop();
    }

    /**
     * Curently this is what we are using but after I get mesurments this should not
     * be used 1/28
     * 
     * @param Joy The joysitck that you are using
     */
    public void JoyClimb1(double Joy, boolean btuon) {
        if (btuon) {
            ClimbingMotor1.getEncoder().setPosition(0);
        }
        if (isClimber1Stopped && isZero(Joy)) {
            return;
        }
        isClimber1Stopped = isZero(Joy);
        ClimbingMotor1.set(Joy);
    }

    public void JoyClimb2(double Joy, boolean btuon) {
        if (btuon) {
            ClimbingMotor2.getEncoder().setPosition(0);
        }
        if (isClimber2Stopped && isZero(Joy)) {
            return;
        }
        isClimber2Stopped = isZero(Joy);
        ClimbingMotor2.set(Joy);
    }

    Timer timer = new Timer();

    public void Flipper(double set) {
        if (isFlipperStopped && isZero(set)) {
            return;
        }
        isFlipperStopped = isZero(set);
        Flipper.set(set);
    }

    public void Stop() {
        // Code to stop the elevator
        ClimbingMotor1.stop();
        ClimbingMotor2.stop();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("CimbEndcoder 1", ClimbingMotor1.getPosition());
        SmartDashboard.putNumber("CimbEndcoder 2", ClimbingMotor2.getPosition());

    }

    private boolean isZero(double value) {
        return Math.abs(value) < 0.00001;
    }

}
