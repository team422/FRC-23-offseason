// package WristTesting;

// import static org.junit.jupiter.api.Assertions.assertEquals;

// import java.beans.Transient;

// import frc.robot.subsystems.wrist.Wrist;
// import org.junit.jupiter.api.AfterEach;
// import org.junit.jupiter.api.BeforeEach;
// import org.junit.jupiter.api.Test;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Rotation2d;
// import frc.robot.subsystems.wrist.WristIOSim;
// import frc.robot.subsystems.wrist.Wrist;
// import frc.robot.Constants;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

// public class WristTest {
//     static final double delta = 1e-2;
//     Wrist m_wrist;
    
//     @BeforeEach
//     void setup() {

//     }

//     void sleepSeconds(int seconds){
//         int i = 0;
//         while(i < seconds * 50){
//             sleep(1);
//             i++;
//         }
//     }

//     void sleep(int ticks) {
//         int i = 0;
//         while (i < ticks) {
//             m_wrist.periodic();
//             m_wrist.simulationPeriodic();
//             i++;
//         }
//     }

//     @Test 
//     void testSpeed() {
    
//     }

//     // @Test
//     // void testBrake(){
//     // m_intake.brake(true);
//     // sleep(1);
//     // assertEquals(true, m_intake.isBrake());
//     // }

// }
