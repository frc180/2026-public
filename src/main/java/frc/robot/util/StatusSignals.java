package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.spamrobotics.util.DisconnectedAlert;

public class StatusSignals {

    private static List<StatusSignal<?>> statusSignals = new ArrayList<>();
    private static StatusSignal<?>[] statusSignalsArray = null;
    private static List<DisconnectedAlert> disconnectedAlerts = new ArrayList<>();

    public static <T> StatusSignal<T> trackSignal(StatusSignal<T> statusSignal) {
        statusSignal.setUpdateFrequency(statusSignal.getAppliedUpdateFrequency());
        statusSignals.add(statusSignal);
        return statusSignal;
    }

    public static void trackSignals(StatusSignal<?>... statusSignals) {
        for (StatusSignal<?> statusSignal : statusSignals) {
            trackSignal(statusSignal);
        }
    }

    /**
     * Calls {@link BaseStatusSignal#refreshAll()} on all tracked signals.
     * @return The StatusCode returned by {@link BaseStatusSignal#refreshAll()}.
     */
    public static StatusCode refreshAll() {
        return BaseStatusSignal.refreshAll(getArray());
    }

    public static void registerAlert(String name, BaseStatusSignal signal) {
        registerAlert(new DisconnectedAlert(name, signal));
    }

    public static void registerAlert(DisconnectedAlert alert) {
        disconnectedAlerts.add(alert);
    }

    public static void updateDisconnectedAlerts() {
        for (DisconnectedAlert alert : disconnectedAlerts) {
            alert.update();
        }
    }

    private static StatusSignal<?>[] getArray() {
        if (statusSignalsArray == null || statusSignalsArray.length != statusSignals.size()) {
            statusSignalsArray = statusSignals.toArray(new StatusSignal<?>[0]);
        }
        return statusSignalsArray;
    }
}
