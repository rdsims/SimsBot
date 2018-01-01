package org.team686.simsbot.vision;

/**
 * A basic interface for classes that get VisionStates. Classes that implement
 * this interface specify what to do when VisionStates are received.
 * 
 * @see VisionState.java
 */
public interface VisionStateListener 
{
    void visionStateNotify();
}