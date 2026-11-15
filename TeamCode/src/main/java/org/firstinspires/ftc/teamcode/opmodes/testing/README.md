# Shooter PIDF test

Select **Shooter PIDF Dashboard Test**, then press START. Set `TARGET_RPM` and
the shooter gains in Panels Configurables. They default to zero. Set RPM to zero
to stop the shooter. Hold gamepad 1 right bumper to run intake at 0.6 power and
transfer at 0.8 power; release to stop both.

To graph shooter speed, add the **Graph** widget to your Panels layout. Under
**Variables**, select **Actual RPM**. Select **Target RPM** too if you want to
compare measured speed against the requested speed. The horizontal axis is time;
both speed traces use RPM. A 15-second window is useful for tuning.

The graph reads the numeric values sent by Panels Telemetry at a configured
50 ms interval (20 Hz); no separate graph API or dependency is needed. The widget
and trace selection are configured in the Panels browser layout. Driver Station
telemetry stays available as well.
