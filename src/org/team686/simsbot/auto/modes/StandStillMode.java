package org.team686.simsbot.auto.modes;

import org.team686.simsbot.auto.AutoModeBase;
import org.team686.simsbot.auto.AutoModeEndedException;

/**
 * Fallback for when all autonomous modes do not work, resulting in a robot
 * standstill
 */
public class StandStillMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Starting Stand Still Mode... Done!");
    }
}