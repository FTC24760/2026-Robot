package org.firstinspires.ftc.teamcode.controls;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * Ties buttons to Ivy commands, since Ivy itself has no gamepad bindings. Register
 * bindings once, then call update() once per loop after Controls.update() and before
 * Scheduler.execute().
 */
public class Bindings {
    private final List<Binding> bindings = new ArrayList<>();

    /** Schedule command when the button goes down and cancel it when it comes up. */
    public void whileHeld(BooleanSupplier button, Command command) {
        bindings.add(new Binding(button) {
            @Override
            void check(boolean pressed, boolean released) {
                if (pressed) Scheduler.schedule(command);
                if (released) Scheduler.cancel(command);
            }
        });
    }

    /** Run action once each time the button goes down. */
    public void onPress(BooleanSupplier button, Runnable action) {
        bindings.add(new Binding(button) {
            @Override
            void check(boolean pressed, boolean released) {
                if (pressed) action.run();
            }
        });
    }

    /** Check every button and act on presses and releases. Call once per loop. */
    public void update() {
        for (Binding binding : bindings) {
            boolean down = binding.button.getAsBoolean();
            binding.check(down && !binding.last, !down && binding.last);
            binding.last = down;
        }
    }

    private abstract static class Binding {
        final BooleanSupplier button;
        boolean last;

        Binding(BooleanSupplier button) {
            this.button = button;
        }

        abstract void check(boolean pressed, boolean released);
    }
}
