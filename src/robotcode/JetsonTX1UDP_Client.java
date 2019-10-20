package robotcode;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;

public class JetsonTX1UDP_Client {
	private DatagramSocket mClientSocket;
	private InetAddress mIPAddress;
	private int mRemotePort;
	private int mBytes;

	public JetsonTX1UDP_Client (String pRemoteAddress, int pRemotePort, int pBytes)
	{
		try {
			mClientSocket = new DatagramSocket();
			mIPAddress = InetAddress.getByName(pRemoteAddress);
			mRemotePort = pRemotePort;
			mBytes = pBytes;
		} catch (SocketException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnknownHostException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public boolean sendPacket (String pData)
	{
		if (pData.length() > mBytes)
		{
			return false;
		}
		DatagramPacket sendPacket = 
				new DatagramPacket (pData.getBytes(), pData.length(), mIPAddress, mRemotePort);
		try {
			mClientSocket.send (sendPacket);
		} catch (@SuppressWarnings("unused") IOException e) {
			return false;
		}
		
		return true;
	}
}
