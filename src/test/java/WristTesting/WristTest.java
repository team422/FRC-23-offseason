package WristTesting;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIOSim;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;

public class WristTest {
    static final double DELTA = Units.degreesToRadians(2);
    Wrist m_wrist;

    void sleep(int seconds) {
        this.sleepCycles(seconds * 50);
    }

    void sleepCycles(int cycles) {
        for (int i = 0; i < cycles; i++) {
            m_wrist.periodic();
            m_wrist.simulationPeriodic();
        }
    }

    @BeforeEach
    public void setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        ProfiledPIDController controller = new ProfiledPIDController(5.5, 0.08, .3, new Constraints(30, 25)); // untuned rn, stole numbers from frc-23
        controller.setTolerance(DELTA);
        m_wrist = new Wrist(new WristIOSim(), controller);
    }

    @Test
    public void setPointTest() {
        Rotation2d[] setPointsRad = {
            Rotation2d.fromDegrees(-90),
            Rotation2d.fromDegrees(-60),
            Rotation2d.fromDegrees(-45),
            Rotation2d.fromDegrees(-20),
            Rotation2d.fromDegrees(-10),
            Rotation2d.fromDegrees(15),
            Rotation2d.fromDegrees(30),
            Rotation2d.fromDegrees(40),
            Rotation2d.fromDegrees(50),
            Rotation2d.fromDegrees(70),
        };

        for (Rotation2d setPoint : setPointsRad) {
            m_wrist.setAngle(setPoint);
            sleep(2);
            // System.out.println(Units.radiansToDegrees(m_wrist.m_inputs.angleRad));
            assertTrue(m_wrist.withinTolerance());
            m_wrist.setAngle(Rotation2d.fromDegrees(0));
            sleep(2);
        }
    }

    @Test
    public void brakeModeTest() {
        m_wrist.setBrakeMode(true);
        sleepCycles(2);
        assertTrue(m_wrist.m_inputs.brake);
    }
}
