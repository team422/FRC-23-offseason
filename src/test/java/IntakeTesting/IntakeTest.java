package IntakeTesting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;

public class IntakeTest {
    static final double DELTA = 1e-4;
    Intake m_intake;

    void sleep(int seconds) {
        sleepCycles(seconds * 50);
    }

    void sleepCycles(int cycles) {
        for (int i = 0; i < cycles; i++) {
            m_intake.periodic();
            m_intake.simulationPeriodic();
        }
    }

    @BeforeEach
    void setup() {
        m_intake = new Intake(new IntakeIOSim());
    }

    @Test
    void testSpeed() {
        double[] speedsToCheck = {-0.9, -0.7, -0.5, -0.3, -0.1, 0.2, 0.4, 0.6, 0.8, 1.0};
        m_intake.setSpeed(speedsToCheck[0]);
        sleep(1);
        double prevSpeed = m_intake.m_inputs.motorSpeed;
        for (int i = 1; i < speedsToCheck.length; i++) {
            double speed = speedsToCheck[i];
            m_intake.setSpeed(speed);
            sleep(1);
            assertTrue(m_intake.m_inputs.motorSpeed > prevSpeed);
            prevSpeed = m_intake.m_inputs.motorSpeed;
            // makes sure works after resetting
            m_intake.setSpeed(0);
            sleep(2);

        }
    }

    @Test
    void brakeModeTest() {
        m_intake.brake(true);
        sleepCycles(2);
        assertTrue(m_intake.m_inputs.brake);
    }
}