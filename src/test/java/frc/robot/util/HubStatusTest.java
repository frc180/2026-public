package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.util.HubStatus.Shift;

public class HubStatusTest {

    // --- Shift.fromTime Tests ---

    @Test
    public void testFromTimeBoundaries() {
        // TRANSITION: (140, 130) -> start inclusive, end exclusive
        assertEquals(Shift.TRANSITION, Shift.fromTime(140));
        assertEquals(Shift.TRANSITION, Shift.fromTime(131));

        // ONE: (130, 105)
        // 130 is not > 130, so Transition fails. 130 <= 130 and > 105, so Shift.ONE.
        assertEquals(Shift.ONE, Shift.fromTime(130));
        assertEquals(Shift.ONE, Shift.fromTime(106));

        // TWO: (105, 80)
        assertEquals(Shift.TWO, Shift.fromTime(105));
        assertEquals(Shift.TWO, Shift.fromTime(81));

        // THREE: (80, 55)
        assertEquals(Shift.THREE, Shift.fromTime(80));
        assertEquals(Shift.THREE, Shift.fromTime(56));

        // FOUR: (55, 30)
        assertEquals(Shift.FOUR, Shift.fromTime(55));
        assertEquals(Shift.FOUR, Shift.fromTime(31));

        // ENDGAME: (30, 0)
        assertEquals(Shift.ENDGAME, Shift.fromTime(30));
        assertEquals(Shift.ENDGAME, Shift.fromTime(1));
    }

    @Test
    public void testFromTimeInvalid() {
        // Logic is > end. For ENDGAME end is 0. So 0 is invalid based on current logic.
        assertEquals(Shift.INVALID, Shift.fromTime(0));
        assertEquals(Shift.INVALID, Shift.fromTime(-1));
        assertEquals(Shift.INVALID, Shift.fromTime(141));
    }

    // --- Alliance Active Logic Tests ---

    @Test
    public void testAllianceActiveBlueWonAuto() {
        boolean isBlue = true;
        boolean blueWonAuto = true;
        
        // TRANSITION/ENDGAME always true
        assertTrue(Shift.TRANSITION.allianceActive(isBlue, blueWonAuto));
        assertTrue(Shift.ENDGAME.allianceActive(isBlue, blueWonAuto));

        // ONE: !allianceWonAuto (False)
        assertFalse(Shift.ONE.allianceActive(isBlue, blueWonAuto)); 
        
        // TWO: allianceWonAuto (True)
        assertTrue(Shift.TWO.allianceActive(isBlue, blueWonAuto));
        
        // THREE: !allianceWonAuto (False)
        assertFalse(Shift.THREE.allianceActive(isBlue, blueWonAuto));
        
        // FOUR: allianceWonAuto (True)
        assertTrue(Shift.FOUR.allianceActive(isBlue, blueWonAuto));
    }

    @Test
    public void testAllianceActiveRedLostAuto() {
        // Red Alliance (isBlue = false)
        // Blue Won Auto (blueWonAuto = true)
        // Did this alliance win auto? No. (false == true -> false)
        
        boolean isBlue = false;
        boolean blueWonAuto = true;

        // ONE: !allianceWonAuto -> !false -> True
        // The loser of auto goes first.
        assertTrue(Shift.ONE.allianceActive(isBlue, blueWonAuto)); 
        
        // TWO: allianceWonAuto -> False
        assertFalse(Shift.TWO.allianceActive(isBlue, blueWonAuto));
    }

    @Test
    public void testAllianceActiveBlueLostAuto() {
        // Blue Alliance (isBlue = true)
        // Red Won Auto (blueWonAuto = false)
        // Did blue win? No.
        
        boolean isBlue = true;
        boolean blueWonAuto = false;

        // ONE: !allianceWonAuto -> !false -> True
        assertTrue(Shift.ONE.allianceActive(isBlue, blueWonAuto));

        // TWO: False
        assertFalse(Shift.TWO.allianceActive(isBlue, blueWonAuto));
    }

    // --- Static Helper Tests ---

    @Test
    public void testStaticAllianceActive() {
        // Time 120 is inside Shift.ONE (130-105).
        // Blue won auto, we are Blue.
        // Shift ONE is for the loser. Blue should be inactive.
        assertFalse(HubStatus.allianceActive(120, true, true));

        // Time 100 is inside Shift.TWO (105-80).
        // Shift TWO is for the winner. Blue should be active.
        assertTrue(HubStatus.allianceActive(100, true, true));
    }

    @Test
    public void testSecondsUntilNextShift() {
        // Time 135 is in TRANSITION (140-130). 
        // End is 130. Result 135 - 130 = 5.
        assertEquals(5, HubStatus.secondsUntilNextShift(135));

        // Time 130 is in ONE (130-105).
        // End is 105. Result 130 - 105 = 25.
        assertEquals(25, HubStatus.secondsUntilNextShift(130));

        // Time 150 is INVALID. Returns -1.
        assertEquals(-1, HubStatus.secondsUntilNextShift(150));
    }
}
