package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Alliance;

@TeleOp(name = "TeleOp: Blue", group = "Competition")
public class TeleOpBlue extends MatchTeleOp {
    public TeleOpBlue() {
        super(Alliance.BLUE);
    }
}