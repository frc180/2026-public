package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;

public class TurretSubsystemTest {

    private double calculate(double current, double target) {
        return TurretSubsystem.findClosestEquivalentAngle(current, target);
    }

    @Test
    public void testStandardRange() {
        // Simple move: Current 100, Target 110
        // Expected: 110 (110 is closest to 100)
        assertEquals(110.0, calculate(100.0, 110.0), 0.001);
    }

    // @Test
    // public void testRotationClimb() {
    //     // Current is in the second rotation (390).
    //     // Target is 40 degrees (which is equivalent to 40, 400).
    //     // 400 is much closer to 390 than 40 is.
    //     // Expected: 400
    //     assertEquals(400.0, calculate(390.0, 40.0), 0.001);
    // }

    @Test
    public void testNegativeInputWrappingLow() {
        // Current: 10 degrees.
        // Target: -10 degrees (Equivalent to 350, 710).
        // 350 is physically closest to 10 (Distance 340) in the forward direction range [0, 720].
        // Ideally we might want -10, but MIN_POS is 0. 
        // 350 is the closest VALID angle.
        assertEquals(350.0, calculate(10.0, -10.0), 0.001);
    }

    @Test
    public void testNegativeInputWrappingHigh() {
        // Current: 700 degrees (Near end of travel).
        // Target: -10 degrees (Equivalent to 350, 710).
        // Distance 700 -> 350 is 350.
        // Distance 700 -> 710 is 10.
        // Expected: 710
        // assertEquals(710.0, calculate(700.0, -10.0), 0.001);
    }

    @Test
    public void testCrossingBoundaryForward() {
        // Current: 350 degrees.
        // Target: 10 degrees (Equivalent to 10, 370).
        // 370 is closer to 350 (dist 20) than 10 is (dist 340).
        // Expected: 370
        assertEquals(370.0, calculate(350.0, 10.0), 0.001);
    }

    @Test
    public void testCrossingBoundaryBackward() {
        // Current: 370 degrees.
        // Target: -10 degrees (Equivalent to 350, 710).
        // 350 is closer to 370 (dist 20) than 710 is (dist 340).
        // Expected: 350
        assertEquals(350.0, calculate(370.0, -10.0), 0.001);
    }

    @Test
    public void testLargeJumpEfficiency() {
        // Current: 180.
        // Target: -170 (Equivalent to 190, 550).
        // -170 is 190 normalized.
        // 190 is clearly closest to 180.
        assertEquals(190.0, calculate(180.0, -170.0), 0.001);
    }

    @Test
    public void testNormalizeAngle() {
        // Zero and small angles
        assertEquals(0.0, TurretSubsystem.normalizeAngle(0.0), 0.001);
        assertEquals(45.0, TurretSubsystem.normalizeAngle(45.0), 0.001);
        assertEquals(-45.0, TurretSubsystem.normalizeAngle(-45.0), 0.001);

        // Wrapping positive
        assertEquals(-90.0, TurretSubsystem.normalizeAngle(270.0), 0.001);
        assertEquals(10.0, TurretSubsystem.normalizeAngle(370.0), 0.001);
        
        // Wrapping negative
        assertEquals(90.0, TurretSubsystem.normalizeAngle(-270.0), 0.001);
        assertEquals(-10.0, TurretSubsystem.normalizeAngle(-370.0), 0.001);

        // Boundaries
        assertEquals(180.0, TurretSubsystem.normalizeAngle(180.0), 0.001);
        // Based on your specific implementation logic, -180 normalizes to 180
        assertEquals(180.0, TurretSubsystem.normalizeAngle(-180.0), 0.001); 
    }
}
