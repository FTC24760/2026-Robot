package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.constants.RobotConstants;

import java.util.List;

/**
 * Owns the REV hubs: manual bulk caching (one sensor read per hub per loop)
 * and a rate-limited battery voltage reading for voltage compensation.
 */
public class HubManager {
    private final List<LynxModule> hubs;
    private final VoltageSensor voltageSensor;

    private long lastVoltageReadNs;
    private double voltageScale = 1.0;

    public HubManager(HardwareMap hardwareMap) {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        voltageSensor = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next()
                : null;
    }

    /** Call exactly once at the top of every loop, before any sensor reads. */
    public void clearBulkCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    /** @return nominal voltage / battery voltage, refreshed every VOLTAGE_READ_PERIOD_MS. */
    public double getVoltageScale(long nowNs) {
        long periodNs = MathUtil.msToNs(RobotConstants.VOLTAGE_READ_PERIOD_MS);

        if (lastVoltageReadNs == 0 || nowNs - lastVoltageReadNs >= periodNs) {
            double voltage = voltageSensor == null ? 0.0 : voltageSensor.getVoltage();
            voltageScale = voltage > 0.1 ? RobotConstants.NOMINAL_VOLTAGE / voltage : 1.0;
            lastVoltageReadNs = nowNs;
        }

        return voltageScale;
    }

    /** Hand the hubs back to automatic caching for whatever OpMode runs next. */
    public void stop() {
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
}
