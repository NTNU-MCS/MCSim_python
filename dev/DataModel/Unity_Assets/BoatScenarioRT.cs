using System;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;
using UnityEditor;

namespace Gemini.EMRS.ScenarioGenerator
{
    public class BoatScenarioRT
    {
        public Vector3 position;
        public Quaternion QuaternionRot;
        public double time; 

        private GameObject _boatObject;

        private UdpClient udpClient; 

        private IPEndPoint RemoteIpEndPoint;
	    private Thread clientReceiveThread;

        public bool running;

        public BoatScenarioRT(GameObject instantiatedBoatPrefab)
        {   
            running = true;
            _boatObject = instantiatedBoatPrefab; 
            time = 0d;
            position = new Vector3(0, 0, 0);
            QuaternionRot = Quaternion.AngleAxis(0, new Vector3(0, 1, 0));
            ConnectToTcpServer ();
        }

        private void ConnectToTcpServer () { 		
		try {  			
			clientReceiveThread = new Thread (new ThreadStart(ListenForData)); 			
			clientReceiveThread.IsBackground = true; 			
			clientReceiveThread.Start();  		
		} 		
		catch (Exception e) { 			
			Debug.Log("On client connect exception " + e); 		
		} 	
	    } 
        private void ListenForData() { 		
		try { 			
			udpClient = new UdpClient(5005);
            RemoteIpEndPoint = new IPEndPoint(IPAddress.Any, 0);    
			while (running) { 				
                Byte[] recievedBytes = udpClient.Receive(ref RemoteIpEndPoint);	
                // Convert byte array to string message. 						
                string serverMessage = Encoding.ASCII.GetString(recievedBytes);
                string[] splitMsg = serverMessage.Split(new char[] { ',' });
                // Parse to usable values
                time = Double.Parse(splitMsg[0]);
                float x = float.Parse(splitMsg[1]);
                float y =  float.Parse(splitMsg[2]);
                float heading =  float.Parse(splitMsg[3]);
                float pitch =  float.Parse(splitMsg[4]); 
                float roll =  float.Parse(splitMsg[5]); 
                float z =  float.Parse(splitMsg[6]);   
                // Set Values
                position = new Vector3(y, z, x);
                QuaternionRot = Quaternion.Euler(new Vector3(pitch, heading, roll)); 	
                Debug.Log("server message received as: " + serverMessage); 
			}         
		}         
		catch (SocketException socketException) {             
			Debug.Log("Socket exception: " + socketException);         
		}
        }

        public void UpdateVessel(){
            _boatObject.transform.position = position; 
            _boatObject.transform.rotation = QuaternionRot;
        }
    }
}