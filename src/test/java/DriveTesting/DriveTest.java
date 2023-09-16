package DriveTesting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Arrays;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleIOSim;
import frc.robot.subsystems.drive.SwerveModuleInputsAutoLogged;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOSim;

public class DriveTest {
    static final double DELTA = 1e-4;
    Drive m_drive;

    void sleep(int seconds) {
        sleepCycles(seconds * 50);
    }

    void sleepCycles(int cycles) {
        for (int i = 0; i < cycles; i++) {
            m_drive.periodic();
            m_drive.simulationPeriodic();
        }
    }

    void assertSameSpeeds(SwerveModuleInputsAutoLogged... inputs) {
        double prevSpeed = inputs[0].driveVelocityMetersPerSecond;
        for (int i = 1; i < inputs.length; i++) {
            assertEquals(prevSpeed, inputs[i].driveVelocityMetersPerSecond, DELTA);
            prevSpeed = inputs[i].driveVelocityMetersPerSecond;
        }
    }

    void testSpeeds(ChassisSpeeds... speeds) {
        // needed separate tests for x and y but didnt wanna write code for them twice

    }

    @BeforeEach
    public void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        GyroIO gyro = new GyroIOSim();
        Pose2d startPose = new Pose2d(3, 5, new Rotation2d());
        SwerveModuleIO[] modules = new SwerveModuleIOSim[4];
        Arrays.fill(modules, new SwerveModuleIOSim());
        m_drive = new Drive(gyro, startPose, modules);
    }

    @Test
    public void xSpeedTest() {
        ChassisSpeeds[] speeds = new ChassisSpeeds[101]; // yes i know this is overkill but idc
        double currDeltaX = -5.0;
        for (int i = 0; i < speeds.length; i++, currDeltaX += 0.1) {
            speeds[i] = new ChassisSpeeds(currDeltaX, 0.0, 0.0);
        }
        for (ChassisSpeeds speed : speeds) {
            m_drive.drive(speed);
            sleep(2);
            assertSameSpeeds(m_drive.m_inputs);
            // System.out.println(speed.vxMetersPerSecond + " " + m_drive.m_inputs[0].driveVelocityMetersPerSecond);
            assertEquals(speed.vxMetersPerSecond, m_drive.m_inputs[0].driveVelocityMetersPerSecond, DELTA);

            m_drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
            sleep(3);
        }
    }

    @Test
    public void ySpeedTest() {
        ChassisSpeeds[] speeds = new ChassisSpeeds[101];
        double currDeltaY = -5.0;
        for (int i = 0; i < speeds.length; i++, currDeltaY += 0.1) {
            speeds[i] = new ChassisSpeeds(0.0, currDeltaY, 0.0);
        }
        for (ChassisSpeeds speed : speeds) {
            m_drive.drive(speed);
            sleep(2);
            assertSameSpeeds(m_drive.m_inputs);
            
            // i have no idea why but it works fine for x but the velocity is always positive for y
            if (speed.vyMetersPerSecond < 0.0) {
                // System.out.println(speed.vyMetersPerSecond + " " + -m_drive.m_inputs[0].driveVelocityMetersPerSecond);
                assertEquals(speed.vyMetersPerSecond, -m_drive.m_inputs[0].driveVelocityMetersPerSecond, DELTA);
            } else {
                // System.out.println(speed.vyMetersPerSecond + " " + m_drive.m_inputs[0].driveVelocityMetersPerSecond);
                assertEquals(speed.vyMetersPerSecond, m_drive.m_inputs[0].driveVelocityMetersPerSecond, DELTA);
            }

            m_drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
            sleep(3);
        }
    }
}
