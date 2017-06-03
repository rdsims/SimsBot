package org.team686.simsbot.vision;

import java.util.List;

/**
 * Tests the vision system by getting targets
 */
public class VisionServerTest
{
	public static class TestReceiver implements VisionStateListener
	{
		@Override
		public void visionStateNotify()
		{
			List<VisionTargetState> targets = VisionState.getInstance().getTargets();
			
			System.out.println("num targets: " + targets.size());
			for (int i = 0; i < targets.size(); i++)
			{
				VisionTargetState target = targets.get(i);
				System.out.println( "Target: " + target.getHorizontalAngle() + ", " + target.getVerticalAngle() + target.getHorizontalWidth() + ", " + target.getVerticalWidth() );
			}
		}
	}

	public static void main(String[] args)
	{
		VisionServer visionServer = VisionServer.getInstance();
		visionServer.addVisionStateReceiver(new TestReceiver());
		while (true)
		{
			try
			{
				Thread.sleep(100);
			}
			catch (InterruptedException e)
			{
				e.printStackTrace();
			}
		}
	}
}
