package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Alliance;

@TeleOp(name = "TeleOp BLUE (full)", group = "Competition")
public class AutoAimBlueTeleOp extends AutoAimTeleOp {
    public AutoAimBlueTeleOp() {
        super(Alliance.BLUE);
    }
}
