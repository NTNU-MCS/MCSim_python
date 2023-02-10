﻿using System;
using System.Globalization;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using Newtonsoft.Json;
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
        public float xRotOffset = 0;
        public float yRotOffset = 0;
        public float zRotOffset = 0;
        private GameObject _boatObject;
        private UdpClient udpClient; 
        private IPEndPoint RemoteIpEndPoint;
	    private Thread clientReceiveThread;
        public bool running;

        public BoatScenarioRT(GameObject instantiatedBoatPrefab, float xRotOffset, float yRotOffset, float zRotOffset)
        {   
            running = true;
            _boatObject = instantiatedBoatPrefab; 
            time = 0d;
            position = new Vector3(0, 0, 0);
            this.xRotOffset = xRotOffset;
            this.yRotOffset = yRotOffset;
            this.zRotOffset = zRotOffset;

            QuaternionRot = Quaternion.Euler(new Vector3(xRotOffset, yRotOffset, zRotOffset));
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
			udpClient = new UdpClient(5000);
            RemoteIpEndPoint = new IPEndPoint(IPAddress.Any, 0);    
			while (running) { 				
                Byte[] recievedBytes = udpClient.Receive(ref RemoteIpEndPoint);	 
                string serverMessage = Encoding.ASCII.GetString(recievedBytes);
                Debug.Log("server message received as: " + serverMessage);
                var details = JsonConvert.DeserializeObject<Dictionary<string, string>>(serverMessage); 
                String msgId = details["message_id"];

                if (String.Equals(msgId, "coords")){
                    float x = float.Parse(details["x"], CultureInfo.InvariantCulture); 
                    float y = float.Parse(details["y"], CultureInfo.InvariantCulture); 
                    float z = float.Parse(details["z"], CultureInfo.InvariantCulture); 
                    position = new Vector3(y, z, x);
                }

                if (String.Equals(msgId, "$PSIMSNS_ext")){
                    float heading =  yRotOffset +  float.Parse(details["head_deg"], CultureInfo.InvariantCulture);
                    float pitch =  xRotOffset + float.Parse(details["pitch_deg"], CultureInfo.InvariantCulture); 
                    float roll =  zRotOffset + float.Parse(details["roll_deg"], CultureInfo.InvariantCulture); 
                    QuaternionRot = Quaternion.Euler(new Vector3(pitch, heading, roll)); 
                } 
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