package robotcode;

import java.io.IOException;
import java.math.BigInteger;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;
import java.util.Arrays;

import constants.ZedAlignmentConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class JetsonTX1UDP_Server {
	private DatagramSocket mSocket;
	private byte[] mReceiveData;
	
	private boolean mValid;
	private double mFB, mLR, mAngle;
	private long mTimestamp;
	
	public JetsonTX1UDP_Server (int pLocalPort, int pBytes)
	{
		try {
			mSocket = new DatagramSocket(pLocalPort);
			mReceiveData = new byte[pBytes];
		} catch (SocketException e) {
			e.printStackTrace();
		}
		
		mValid = false;
		mFB = 10;
		mLR = 0;
		mAngle = 0;
		mTimestamp = 0;
		
		StringBuilder headerString = new StringBuilder();
		for (int i = 0; i < ZedAlignmentConstants.RobotPositionData.length; i++)
		{
			if (i > 0)
			{
				headerString.append(",");
			}
			headerString.append (ZedAlignmentConstants.RobotPositionData[i]);
		}
		
	}
	
	
	private String recievePacket ()
	{
		try {
			Arrays.fill (mReceiveData, (byte) 0);
			DatagramPacket receivePacket = new DatagramPacket (mReceiveData, mReceiveData.length);
			mSocket.receive(receivePacket);
			String data = new String (receivePacket.getData());
			return data;
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			return "ERROR";
		}
	}
	
	public void close()
	{
		mSocket.close();
	}

	public synchronized boolean getValid ()
	{
		return mValid;
	}
	
	
	public synchronized long getTimestamp ()
	{
		return mTimestamp;
	}
	
	public synchronized double getFB ()
	{
		return mFB;
	}
	
	public synchronized double getLR()
	{
		return mLR;
	}
	
	public synchronized double getAngle()
	{
		return mAngle;
	}
	
	public void updateData()
	{
		SmartDashboard.putString ("Attempting to read data", "YES1");
		String data = recievePacket();
		SmartDashboard.putString ("TX1 Data:", data);
		try 
		{	
			String[] pieces = data.split(",");
			if (pieces[0].equals ("SUCCESS"))
			{
				long timedif = Long.parseLong ((new BigInteger (pieces[1]).toString()));
				long timestamp = System.currentTimeMillis() - timedif;
				//SmartDashboard.putString ("LR Piece:", pieces[2]);
				double lr = 
						Double.parseDouble(pieces[2]);

				double fb = 
						Double.parseDouble(pieces[3]);

				double angle = 
						Double.parseDouble(pieces[4]);

				mTimestamp = timestamp;
				mLR = lr;
				mFB = fb;
				mAngle = angle;
				mValid = true;
			}
			else
			{
				mLR = 0;
				mFB = 30;
				mAngle = 0;
				mValid = false;
			}
		}
		catch (Exception e)
		{
			mLR = 0;
			mFB = 30;
			mAngle = 0;
			mValid = false;
			SmartDashboard.putString("Parsing Peg Data Exception:", e.getMessage());
		}	
		
		SmartDashboard.putBoolean("HasGoodTarget", mValid);
	}
}
