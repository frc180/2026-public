package com.spamrobotics.util;

import java.util.function.Supplier;
import frc.robot.Robot;

public class AllianceZone implements Supplier<Zone> {

    final Zone blue, red;

    public AllianceZone(Zone blue, Zone red) {
        this.blue = blue;
        this.red = red;
    }

    @Override
    public Zone get() {
        return Robot.isBlue() ? blue : red;
    }

    public static AllianceZone fromBlue(Zone blue) {
        return new AllianceZone(blue, blue.allianceFlip());
    }

    public static AllianceZone fromRed(Zone red) {
        return new AllianceZone(red.allianceFlip(), red);
    }
}
