package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Alliance;

@TeleOp(name = "TeleOp RED (full)", group = "Competition")
public class AutoAimRedTeleOp extends AutoAimTeleOp {
    public AutoAimRedTeleOp() {
        super(Alliance.RED);
    }
}
