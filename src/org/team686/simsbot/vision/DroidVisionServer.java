package org.team686.simsbot.vision;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.*;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;

import org.team686.lib.util.CrashTrackingRunnable;
import org.team686.simsbot.Constants;
import org.team686.simsbot.vision.messages.HeartbeatMessage;
import org.team686.simsbot.vision.messages.OffWireMessage;
import org.team686.simsbot.vision.messages.VisionMessage;

/**
 * This controls all vision actions, including vision updates, capture, and
 * interfacing with the Android phone with Android Debug Bridge. It also stores
 * all VisionStates (from the Android phone) and contains methods to add
 * to/prune the VisionState list. Much like the subsystems, outside methods get
 * the VisionServer instance (there is only one VisionServer) instead of
 * creating new VisionServer instances.
 * 
 * @see VisionState.java
 */

public class DroidVisionServer extends CrashTrackingRunnable
{
	private static DroidVisionServer instance = null;
	private ServerSocket serverSocket;
	private boolean running = true;
	private int port;
	AdbBridge adb = new AdbBridge();
	double lastMessageReceivedTime = 0;
	private boolean useJavaTime = false;

	private ArrayList<ServerThread> serverThreads = new ArrayList<>();
	private volatile boolean appRestartRequested = false;

	public static DroidVisionServer getInstance()
	{
		if (instance == null)
		{
			instance = new DroidVisionServer(Constants.kAndroidAppTcpPort);
		}
		return instance;
	}

	private boolean connected = false;

	public boolean isConnected()
	{
		return connected;
	}

	public void requestAppRestart()
	{
		appRestartRequested = true;
	}

	/**
	 * Instantializes the VisionServer and connects to ADB via the specified
	 * port.
	 * 
	 * @param Port
	 */
	private DroidVisionServer(int _port)
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

	protected class ServerThread extends CrashTrackingRunnable
	{
		private Socket socket;

		public ServerThread(Socket _socket)
		{
			socket = _socket;
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
				int numBytesRead;
				// numBytesRead==-1 when end of stream has been reached
				while (socket.isConnected() && (numBytesRead = is.read(buffer)) != -1) 
				{
					double timestamp = getTimestamp();
					lastMessageReceivedTime = timestamp;
					String messageRaw = new String(buffer, 0, numBytesRead);

					// if multiple messages have been received, they will be
					// separated by "\n"s
					String[] messages = messageRaw.split("\n");
					for (String message : messages)
					{
						// first level of parsing into "type" and "message"
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

		public void handleMessage(double timestamp, VisionMessage message)
		{
			if ("targets".equals(message.getType()))
			{
				VisionState.getInstance().updateFromJsonString(timestamp, message.getMessage());
			}

			if ("heartbeat".equals(message.getType()))
			{
				send(HeartbeatMessage.getInstance());
			}
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
					connected = false;
				}
				else
				{
					connected = true;
				}
				if (appRestartRequested)
				{
					restartApp();
					appRestartRequested = false;
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

		public void restartApp()
		{
			adb.stopPackage(Constants.kAppPackage);
			adb.startActivity(Constants.kAppActivity);
		}

	}

}
