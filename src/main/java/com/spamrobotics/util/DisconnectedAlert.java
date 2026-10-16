package com.spamrobotics.util;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.wpilibj.Alert;

public class DisconnectedAlert extends Alert {

    private final BaseStatusSignal signal;

    public DisconnectedAlert(String name, BaseStatusSignal signal) {
        super(name + " disconnected", AlertType.kError);
        this.signal = signal;
    }

    public void update() {
        set(signal.getTimestamp().getLatency() > 0.5);
    }
}
