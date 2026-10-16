package frc.robot.util;

public class HubStatus {

    public enum Shift {
        TRANSITION(140, 130),
        ONE(130, 105),
        TWO(105, 80),
        THREE(80, 55),
        FOUR(55, 30),
        ENDGAME(30, 0),
        INVALID(-1, -1);

        public final int start;
        public final int end;

        Shift(int start, int end) {
            this.start = start;
            this.end = end;
        }

        public boolean allianceActive(boolean blue, boolean blueWonAuto) {
            boolean allianceWonAuto = blueWonAuto == blue;
            switch (this) {
                case TRANSITION, ENDGAME, INVALID:
                    return true;
                case ONE:
                    return !allianceWonAuto;
                case TWO:
                    return allianceWonAuto;
                case THREE:
                    return !allianceWonAuto;
                case FOUR:
                    return allianceWonAuto;
                default:
                    return true;
            }
        } 

        public static Shift fromTime(double timeRemainingSeconds) {
            for (Shift shift : Shift.values()) {
                if (timeRemainingSeconds <= shift.start && timeRemainingSeconds > shift.end) {
                    return shift;
                }
            }
            return INVALID;
        }
    }

    public static boolean allianceActive(double timeRemainingSeconds, boolean blue, boolean blueWonAuto) {
        return Shift.fromTime(timeRemainingSeconds).allianceActive(blue, blueWonAuto);
    }

    public static double secondsUntilNextShift(double timeRemainingSeconds) {
        Shift currentShift = Shift.fromTime(timeRemainingSeconds);
        if (currentShift == Shift.INVALID) {
            return -1;
        }
        return timeRemainingSeconds - currentShift.end;
    }

    public static double secondsUntilNextActiveShift(double timeRemainingSeconds, boolean blue, boolean blueWonAuto) {
        Shift currentShift = Shift.fromTime(timeRemainingSeconds);
        if (currentShift == Shift.INVALID || currentShift.allianceActive(blue, blueWonAuto)) {
            return -1;
        }

        return secondsUntilNextShift(timeRemainingSeconds);
    }

    public static double secondsUntilNextInactiveShift(double timeRemainingSeconds, boolean blue, boolean blueWonAuto) {
        Shift currentShift = Shift.fromTime(timeRemainingSeconds);
        if (currentShift == Shift.INVALID || !currentShift.allianceActive(blue, blueWonAuto)) {
            return -1;
        }


        // Find the next shift that is inactive for the alliance
        double time = timeRemainingSeconds;
        while (true) {
            // Calculate time remaining in the current shift logic loop is examining
            double secondsToNextShift = secondsUntilNextShift(time);
            time -= secondsToNextShift;

            // Identify the shift that follows immediately
            Shift nextShift = Shift.fromTime(time);
            if (nextShift == Shift.INVALID) {
                return -1;
            }

            // If that next shift is inactive, we are done; the time until then is the difference
            if (!nextShift.allianceActive(blue, blueWonAuto)) {
                return timeRemainingSeconds - time;
            }
        }
    }
}
