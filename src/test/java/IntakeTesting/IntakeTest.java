package IntakeTesting;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
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
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        m_intake = new Intake(new IntakeIOSim());
    }

    @Test
    void testSpeed() {
        double[] speedsToCheck = {-0.9, -0.7, -0.5, -0.3, -0.1, 0.2, 0.4, 0.6, 0.8, 1.0};
        m_intake.setVoltage(speedsToCheck[0]);
        sleep(1);
        double prevSpeed = m_intake.m_inputs.intakeSpeed;
        for (int i = 1; i < speedsToCheck.length; i++) {
            double speed = speedsToCheck[i];
            m_intake.setVoltage(speed);
            sleep(1);
            assertTrue(m_intake.m_inputs.intakeSpeed > prevSpeed);
            prevSpeed = m_intake.m_inputs.intakeSpeed;
            // makes sure works after resetting
            m_intake.setVoltage(0);
            sleep(2);

        }
    }
}