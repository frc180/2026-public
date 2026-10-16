package com.spamrobotics.util;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemExt extends SubsystemBase {

    public static final List<SubsystemExt> ALL_SUBSYSTEMS = new ArrayList<>();

    public SubsystemExt() {
        super();
        ALL_SUBSYSTEMS.add(this);
    }

    public void periodicAfterScheduler() {
        // Override this method to run code after the Command scheduler runs.
    }
}
