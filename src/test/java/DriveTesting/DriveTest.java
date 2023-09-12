package DriveTesting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.beans.Transient;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveModuleIOSim;
import frc.robot.subsystems.drive.accelerometer.AccelerometerIOSim;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class DriveTest {
    static final double delta = 1e-2;
    Drive m_drive;

    @BeforeEach
    void setup() {
        m_drive = new Drive(new GyroIOPigeon(22, new Rotation2d()), new AccelerometerIOSim(), new Pose2d(),
        new SwerveModuleIOSim(), new SwerveModuleIOSim(), new SwerveModuleIOSim(), new SwerveModuleIOSim());
    }

    void sleepSeconds(int seconds) {
        for (int i = 0; i < seconds; i++) {
            sleep(1);
        }
    }

    void sleep(int ticks) {
        int i = 0;
        while (i < ticks) {
            m_drive.periodic();
            m_drive.simulationPeriodic();
            i++;
        }
    }



}
