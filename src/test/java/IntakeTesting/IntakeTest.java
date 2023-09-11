import static org.junit.jupiter.api.Assertions.assertEquals;

import java.beans.Transient;

import frc.robot.subsystems.intake.Intake;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;


public class IntakeTest {
    static final double delta = 1e-2;
    Intake m_intake;
    
    @BeforeEach
    void setup() {
        m_intake = new Intake();
    }

    void sleep(double seconds){
        int i = 0;
        while(i < seconds * 50){
            m_intake.periodic();
            m_intake.simulationPeriodic();
            i++;
        }
    }

    @Test 
    void testSpeed() {
        double prevSpeed = -1e10;
        for (double i = -1; i < 1; i += 0.1) {
            m_intake.setSpeed(i);
            sleep(1);
            assertEquals(m_intake.getSpeed() > prevSpeed, true);
            prevSpeed = m_intake.getSpeed();
            System.out.println(prevSpeed + "=speed, i=" + i);
        }
    }

    // @Test
    // void testBrake(){
    //     m_intake.brake(true);
    //     sleep(0.02);
    //     assertEquals(true, m_intake.isBrake());
    // }

}
