package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lidar extends SubsystemBase {

    private Counter m_LIDAR;

    public Lidar() {
        super();
        m_LIDAR = new Counter(5); // plug the lidar into PWM 0
        m_LIDAR.setMaxPeriod(1.00); // set the max period that can be measured
        m_LIDAR.setSemiPeriodMode(true); // Set the counter to period measurement
        m_LIDAR.reset();
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Lidar", getDistanceMeters());
    }

    public double getDistanceMeters() {
        return m_LIDAR.getPeriod() * 1000000.0 / 1000.0;
    }

}
