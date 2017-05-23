package org.team686.simsbot.vision;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.*;
import java.util.ArrayList;
import java.util.Collections;

import edu.wpi.first.wpilibj.Timer;

import org.team686.lib.util.CrashTrackingRunnable;
import org.team686.simsbot.Constants;
import org.team686.simsbot.vision.messages.HeartbeatMessage;
import org.team686.simsbot.vision.messages.OffWireMessage;
import org.team686.simsbot.vision.messages.VisionMessage;

/**
 * This controls all vision actions, including vision updates, capture, and
 * interfacing with the Android phone with Android Debug Bridge. It also stores
 * all VisionUpdates (from the Android phone) and contains methods to add
 * to/prune the VisionUpdate list. Much like the subsystems, outside methods get
 * the VisionServer instance (there is only one VisionServer) instead of
 * creating new VisionServer instances.
 * 
 * @see VisionUpdate.java
 */

public class VisionServer extends CrashTrackingRunnable
{

	private static VisionServer instance = null;
	private ServerSocket serverSocket;
	private boolean running = true;
	private int port;
	private ArrayList<VisionUpdateReceiver> receivers = new ArrayList<>();
	AdbBridge adb = new AdbBridge();
	double lastMessageReceivedTime = 0;
	private boolean useJavaTime = false;

	private ArrayList<ServerThread> serverThreads = new ArrayList<>();
	private volatile boolean mWantsAppRestart = false;

	public static VisionServer getInstance()
	{
		if (instance == null)
		{
			instance = new VisionServer(Constants.kAndroidAppTcpPort);
		}
		return instance;
	}

	private boolean mIsConnect = false;

	public boolean isConnected()
	{
		return mIsConnect;
	}

	public void requestAppRestart()
	{
		mWantsAppRestart = true;
	}

	protected class ServerThread extends CrashTrackingRunnable
	{
		private Socket socket;

		public ServerThread(Socket _socket)
		{
			socket = _socket;
		}

		public void send(VisionMessage message)
		{
			String toSend = message.toJson() + "\n";
			if (socket != null && socket.isConnected())
			{
				try
				{
					OutputStream os = socket.getOutputStream();
					os.write(toSend.getBytes());
				}
				catch (IOException e)
				{
					System.err.println("VisionServer: Could not send data to socket");
				}
			}
		}

		public void handleMessage(double timestamp, VisionMessage message)
		{
			if ("targets".equals(message.getType()))
			{
				VisionUpdate update = VisionUpdate.generateFromJsonString(timestamp, message.getMessage());
				receivers.removeAll(Collections.singleton(null));
				if (update.isValid())
				{
					for (VisionUpdateReceiver receiver : receivers)
					{
						receiver.gotUpdate(update);
					}
				}
			}
			if ("heartbeat".equals(message.getType()))
			{
				send(HeartbeatMessage.getInstance());
			}
		}

		public boolean isAlive()
		{
			return socket != null && socket.isConnected() && !socket.isClosed();
		}

		@Override
		public void runCrashTracked()
		{
			if (socket == null)
			{
				return;
			}
			try
			{
				InputStream is = socket.getInputStream();
				byte[] buffer = new byte[2048];
				int read;
				while (socket.isConnected() && (read = is.read(buffer)) != -1)
				{
					double timestamp = getTimestamp();
					lastMessageReceivedTime = timestamp;
					String messageRaw = new String(buffer, 0, read);
					String[] messages = messageRaw.split("\n");
					for (String message : messages)
					{
						OffWireMessage parsedMessage = new OffWireMessage(message);
						if (parsedMessage.isValid())
						{
							handleMessage(timestamp, parsedMessage);
						}
					}
				}
				System.out.println("Socket disconnected");
			}
			catch (IOException e)
			{
				System.err.println("Could not talk to socket");
			}
			if (socket != null)
			{
				try
				{
					socket.close();
				}
				catch (IOException e)
				{
					e.printStackTrace();
				}
			}
		}
	}

	/**
	 * Instantializes the VisionServer and connects to ADB via the specified
	 * port.
	 * 
	 * @param Port
	 */
	private VisionServer(int _port)
	{
		try
		{
			adb = new AdbBridge();
			port = _port;
			serverSocket = new ServerSocket(port);
			adb.start();
			adb.reversePortForward(port, port);
			try
			{
				useJavaTime = System.getenv("USE_JAVA_TIME").equals("true");
			}
			catch (NullPointerException e)
			{
				useJavaTime = false;
			}
		}
		catch (IOException e)
		{
			e.printStackTrace();
		}
		new Thread(this).start();
		new Thread(new AppMaintainanceThread()).start();
	}

	public void restartAdb()
	{
		adb.restartAdb();
		adb.reversePortForward(port, port);
	}

	/**
	 * If a VisionUpdate object (i.e. a target) is not in the list, add it.
	 * 
	 * @see VisionUpdate
	 */
	public void addVisionUpdateReceiver(VisionUpdateReceiver receiver)
	{
		if (!receivers.contains(receiver))
		{
			receivers.add(receiver);
		}
	}

	public void removeVisionUpdateReceiver(VisionUpdateReceiver receiver)
	{
		if (receivers.contains(receiver))
		{
			receivers.remove(receiver);
		}
	}

	@Override
	public void runCrashTracked()
	{
		while (running)
		{
			try
			{
				Socket p = serverSocket.accept();
				ServerThread s = new ServerThread(p);
				new Thread(s).start();
				serverThreads.add(s);
			}
			catch (IOException e)
			{
				System.err.println("Issue accepting socket connection!");
			}
			finally
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

	private class AppMaintainanceThread extends CrashTrackingRunnable
	{
		@Override
		public void runCrashTracked()
		{
			while (true)
			{
				if (getTimestamp() - lastMessageReceivedTime > .1)
				{
					// camera disconnected
					adb.reversePortForward(port, port);
					mIsConnect = false;
				}
				else
				{
					mIsConnect = true;
				}
				if (mWantsAppRestart)
				{
					adb.restartApp();
					mWantsAppRestart = false;
				}
				try
				{
					Thread.sleep(200);
				}
				catch (InterruptedException e)
				{
					e.printStackTrace();
				}
			}
		}
	}

	private double getTimestamp()
	{
		if (useJavaTime)
		{
			return System.currentTimeMillis();
		}
		else
		{
			return Timer.getFPGATimestamp();
		}
	}
}
