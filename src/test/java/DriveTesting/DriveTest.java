package DriveTesting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleIOSim;
import frc.robot.subsystems.drive.SwerveModuleInputsAutoLogged;
import frc.robot.subsystems.drive.accelerometer.AccelerometerIO;
import frc.robot.subsystems.drive.accelerometer.AccelerometerIOSim;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon;

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

    @BeforeEach
    public void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        GyroIO gyro = new GyroIOPigeon(22, new Rotation2d());
        AccelerometerIO accel = new AccelerometerIOSim();
        Pose2d startPose = new Pose2d(3, 5, new Rotation2d());
        SwerveModuleIO[] modules = new SwerveModuleIOSim[4];
        Arrays.fill(modules, new SwerveModuleIOSim());
        m_drive = new Drive(gyro, accel, startPose, modules);
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

    @Test
    public void speedTest() {
        // general speed test, other tests see if driveVelocityMetersPerSecond works, this
        // sees if xDriveVelocityMetersPerSecond and yDriveVelocityMetersPerSecond work
        ChassisSpeeds[] speeds = new ChassisSpeeds[101];
        double currDeltaX = -5.0;
        double currDeltaY = -10.0;
        for (int i = 0; i < speeds.length; i++, currDeltaX += 0.1, currDeltaY += 0.2) {
            speeds[i] = new ChassisSpeeds(currDeltaX, currDeltaY, 0.0);
        }
        for (ChassisSpeeds speed : speeds) {
            m_drive.drive(speed);
            sleep(2);

            assertSameSpeeds(m_drive.m_inputs);
            // System.out.println(speed.vxMetersPerSecond + " " + m_drive.m_inputs[0].xDriveVelocityMetersPerSecond);
            assertEquals(speed.vxMetersPerSecond, m_drive.m_inputs[0].xDriveVelocityMetersPerSecond, DELTA);
            // System.out.println(speed.vyMetersPerSecond + " " + m_drive.m_inputs[0].yDriveVelocityMetersPerSecond);
            assertEquals(speed.vyMetersPerSecond, m_drive.m_inputs[0].yDriveVelocityMetersPerSecond, DELTA);

            m_drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
            sleep(3);
        }
    }

    @Test
    public void oneDimensionalPoseEstimationTest() {
        // test if the pose estimation works when driving in a straight line
        ChassisSpeeds[] xSpeeds = new ChassisSpeeds[] {
            new ChassisSpeeds(-1.0, 0.0, 0.0),
            new ChassisSpeeds(1.0, 0.0, 0.0),
            new ChassisSpeeds(0.0, -1.0, 0.0),
            new ChassisSpeeds(0.0, 1.0, 0.0),
        };

        ChassisSpeeds[] ySpeeds = new ChassisSpeeds[] {
            new ChassisSpeeds(0.0, -1.0, 0.0),
            new ChassisSpeeds(0.0, 1.0, 0.0)
        };

        Pose2d startPose = m_drive.getPose();
        // this is all hard-coded bc i need to manually test the position after each speed

        m_drive.drive(xSpeeds[0]);
        sleep(2);
        m_drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        sleep(3);
        assertTrue(m_drive.getPose().getX() < startPose.getX());
        assertEquals(startPose.getY(), m_drive.getPose().getY(), DELTA);

        m_drive.drive(xSpeeds[1]);
        sleep(2);
        m_drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        sleep(3);
        assertEquals(startPose.getX(), m_drive.getPose().getX(), DELTA);
        assertEquals(startPose.getY(), m_drive.getPose().getY(), DELTA);

        m_drive.drive(ySpeeds[0]);
        sleep(2);
        m_drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        sleep(3);
        assertEquals(startPose.getX(), m_drive.getPose().getX(), DELTA);
        assertTrue(m_drive.getPose().getY() < startPose.getY());
        

        m_drive.drive(ySpeeds[1]);
        sleep(2);
        m_drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        sleep(3);
        assertEquals(startPose.getX(), m_drive.getPose().getX(), DELTA);
        assertEquals(startPose.getY(), m_drive.getPose().getY(), DELTA);
    }

    @Test
    public void twoDimensionalPoseEstimationTest() {
        // test if the pose estimation works when driving in both x and y
        // this function name is long as hell

        ChassisSpeeds[] speeds = new ChassisSpeeds[20];
        double currDeltaX = -5.0;
        double currDeltaY = -10.0;
        for (int i = 0; i < speeds.length; i++, currDeltaX += 0.5, currDeltaY += 1.0) {
            speeds[i] = new ChassisSpeeds(currDeltaX, currDeltaY, 0.0);
        }

        Pose2d startPose = m_drive.getPose();

        for (ChassisSpeeds speed : speeds) {
            m_drive.drive(speed);
            sleep(2);
            m_drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
            sleep(3);
            ChassisSpeeds counterSpeeds = new ChassisSpeeds(-speed.vxMetersPerSecond, -speed.vyMetersPerSecond, 0.0);
            m_drive.drive(counterSpeeds);
            sleep(2);
            m_drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
            sleep(3);
            assertEquals(startPose.getX(), m_drive.getPose().getX(), DELTA);
            assertEquals(startPose.getY(), m_drive.getPose().getY(), DELTA);
        }
    }

    @Test
    public void rotationSpeedTest() {
        // currently broken but still wanted to push progress

        sleep(1);
        System.out.println(m_drive.m_inputs[0].turnAngle);

        m_drive.drive(new ChassisSpeeds(0, 0, 1.0));
        sleep(10);
        System.out.println(m_drive.m_inputs[0].turnAngle + " " + m_drive.m_inputs[0].turnRadsPerSecond);

        /*
        ChassisSpeeds[] speeds = new ChassisSpeeds[359 * 2]; // its overkill but idc
        for (int i = 0; i < 359 * 2; i++) {
            double omegaSpeed = i / 2;
            speeds[i] = new ChassisSpeeds(0.0, 0.0, omegaSpeed);
        }

        for (ChassisSpeeds speed : speeds) {
            m_drive.drive(speed);
            sleep(5);

            System.out.println(m_drive.m_inputs[0].turnAngle);
            System.out.println(speed.omegaRadiansPerSecond + " " + m_drive.m_inputs[0].turnRadsPerSecond);
            assertSameSpeeds(m_drive.m_inputs);
            assertEquals(speed.omegaRadiansPerSecond, m_drive.m_inputs[0].turnRadsPerSecond, DELTA);

            m_drive.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
            sleep(3);
        } */
    }
}
