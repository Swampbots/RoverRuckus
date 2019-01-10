package org.firstinspires.ftc.teamcode;

import java.util.LinkedList;
import java.util.List;

public class GamepadCooldowns {

    ButtonCooldown dpUp     = new ButtonCooldown();
    ButtonCooldown dpDown   = new ButtonCooldown();
    ButtonCooldown dpLeft   = new ButtonCooldown();
    ButtonCooldown dpRight  = new ButtonCooldown();

    ButtonCooldown a    = new ButtonCooldown();
    ButtonCooldown b    = new ButtonCooldown();
    ButtonCooldown x    = new ButtonCooldown();
    ButtonCooldown y    = new ButtonCooldown();

    ButtonCooldown lb   = new ButtonCooldown();
    ButtonCooldown rb   = new ButtonCooldown();
    ButtonCooldown lt   = new ButtonCooldown();
    ButtonCooldown rt   = new ButtonCooldown();

    List<ButtonCooldown> cooldowns = new LinkedList<>();

    public GamepadCooldowns() {
        cooldowns.add(dpUp);
        cooldowns.add(dpDown);
        cooldowns.add(dpLeft);
        cooldowns.add(dpRight);

        cooldowns.add(a);
        cooldowns.add(b);
        cooldowns.add(x);
        cooldowns.add(y);

        cooldowns.add(lb);
        cooldowns.add(rb);
        cooldowns.add(lt);
        cooldowns.add(rt);
    }

        for(ButtonCooldown button : cooldowns) {
            button.setCooldown(millis);
    public void setCooldown(double cooldown) {
            button.setCooldown(cooldown);
        }
    }




}
