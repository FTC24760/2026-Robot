package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoContext;
import org.firstinspires.ftc.teamcode.auto.AutoOpMode;
import org.firstinspires.ftc.teamcode.auto.AutoRoutine;
import org.firstinspires.ftc.teamcode.auto.routines.FullAuto;
import org.firstinspires.ftc.teamcode.utils.Alliance;

@Autonomous(name = "Auto: Blue", group = "Competition", preselectTeleOp = "TeleOp: Blue")
public class AutoBlue extends AutoOpMode {
    public AutoBlue() {
        super(Alliance.BLUE);
    }

    @Override
    protected AutoRoutine buildRoutine(AutoContext ctx) {
        return FullAuto.build(ctx);
    }
}
