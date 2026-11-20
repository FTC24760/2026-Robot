package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Alliance;

@TeleOp(name = "TeleOp: Red", group = "Competition")
public class TeleOpRed extends MatchTeleOp {
    public TeleOpRed() {
        super(Alliance.RED);
    }
}
