package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoContext;
import org.firstinspires.ftc.teamcode.auto.AutoOpMode;
import org.firstinspires.ftc.teamcode.auto.AutoRoutine;
import org.firstinspires.ftc.teamcode.auto.routines.FullAuto;
import org.firstinspires.ftc.teamcode.utils.Alliance;

@Autonomous(name = "Auto: Red", group = "Competition", preselectTeleOp = "TeleOp: Red")
public class AutoRed extends AutoOpMode {
    public AutoRed() {
        super(Alliance.RED);
    }

    @Override
    protected AutoRoutine buildRoutine(AutoContext ctx) {
        return FullAuto.build(ctx);
    }
}
