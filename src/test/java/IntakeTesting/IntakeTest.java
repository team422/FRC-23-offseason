package IntakeTesting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.beans.Transient;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSim;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;


public class IntakeTest {
    static final double delta = 1e-2;
    Intake m_intake;
    
    @BeforeEach
    void setup() {
        m_intake = new Intake(new IntakeIOSim());
    }

    void sleepSeconds(int seconds){
        int i = 0;
        while(i < seconds * 50){
            sleep(1);
            i++;
        }
    }

    void sleep(int ticks) {
        int i = 0;
        while (i < ticks) {
            m_intake.periodic();
            m_intake.simulationPeriodic();
            i++;
        }
    }

    @Test 
    void testSpeed() {
        double prevSpeed = -1e10;
        for (double i = -12; i < 12; i++) {
            m_intake.setDesiredVoltage(i);
            sleepSeconds(2);
            assertEquals(m_intake.getSpeed() > prevSpeed, true);
            prevSpeed = m_intake.getSpeed();
        }
    }

    // @Test
    // void testBrake(){
    //     m_intake.brake(true);
    //     sleep(1);
    //     assertEquals(true, m_intake.isBrake());
    // }

}
