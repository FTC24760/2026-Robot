package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utils.Alliance;

@Autonomous(name = "Auto: Rapid Fire Red (sample)", group = "Competition")
public class RapidFireRedAuto extends RapidFireAuto {
    public RapidFireRedAuto() {
        super(Alliance.RED);
    }
}
