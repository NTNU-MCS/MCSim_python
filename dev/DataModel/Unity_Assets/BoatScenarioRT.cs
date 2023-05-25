using System;
using System.Globalization;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using Newtonsoft.Json; 
using System.Text; 
using WebSocketSharp;
using UnityEngine;
using UnityEditor;
using System.Text.RegularExpressions;

namespace Gemini.EMRS.ScenarioGenerator
{
    public class BoatScenarioRT
    {
        [System.Serializable]
        public class InMessage
        { 
            public string type; 
            public Dictionary<string, string> data;
        }
        public Vector3 position;
        public Quaternion QuaternionRot;
        public double time; 
        public float xRotOffset = 0;
        public float yRotOffset = 0;
        public float zRotOffset = 0;
        private GameObject _boatObject;
        public AisBoat[] aisObjects = new AisBoat[]{};  
        public bool running;

        private WebSocket ws;

        public String mmsiGunnerus = "258342000";

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
            ListenForData();
        } 

        private void ListenForData() { 	 
			ws = new WebSocket("ws://127.0.0.1:8000");
            ws.Connect();
            ws.OnMessage += (sender, e) =>
            {  
                try {
                var content = Encoding.UTF8.GetString(e.RawData);
                string pattern =  @"""data"":\s*\{([^}]+)\}";  
                MatchCollection matches = Regex.Matches(content, pattern);  
                string extractedText = "{"+matches[0].Groups[1].Value+"}"; 
                var details = JsonConvert.DeserializeObject<Dictionary<string, string>>(extractedText);  
                String msgId = details["message_id"];
                
                if (String.Equals(msgId, "ais")){
                        setAisData(details);
                    }

                if (String.Equals(msgId, "coords")){
                    float x = float.Parse(details["x"], CultureInfo.InvariantCulture); 
                    float y = float.Parse(details["y"], CultureInfo.InvariantCulture); 
                    float z = float.Parse(details["z"], CultureInfo.InvariantCulture); 
                    position = new Vector3(x, z, y);
                }

                if (String.Equals(msgId, "$PSIMSNS_ext")){
                    float heading =  yRotOffset +  float.Parse(details["head_deg"], CultureInfo.InvariantCulture);
                    float pitch =  xRotOffset + float.Parse(details["pitch_deg"], CultureInfo.InvariantCulture); 
                    float roll =  zRotOffset + float.Parse(details["roll_deg"], CultureInfo.InvariantCulture); 
                    QuaternionRot = Quaternion.Euler(new Vector3(pitch, heading, roll)); 
                }  
                } catch {}
		};         
		 
        }

        private void setAisData(Dictionary<string,string> message) {
            string mmsi = message["mmsi"];
            float x = float.Parse(message["x"], CultureInfo.InvariantCulture); 
            float y = float.Parse(message["y"], CultureInfo.InvariantCulture);  
            Vector3 pos = new Vector3(x, 0f, y);

            float heading = float.Parse(message["heading"], CultureInfo.InvariantCulture); 
            Quaternion rot = Quaternion.Euler(new Vector3(0f, heading, 0f));
            int index = Array.FindIndex(aisObjects,i => i.id == mmsi);
            bool isGunnerus = mmsi.Equals(mmsiGunnerus);
            if (!isGunnerus) {
                if (index == -1) {
                    int newLength = aisObjects.Length + 1;
                    Debug.Log("lengt: " + aisObjects.Length); 
                    Array.Resize(ref aisObjects, newLength);
                    Debug.Log("lengt a: " + aisObjects.Length); 
                    aisObjects[aisObjects.Length-1] = new AisBoat(mmsi);
                } else { 
                    aisObjects[index].pos = pos; 
                    aisObjects[index].rot = rot;
                }
            }
        }

        public void UpdateVessel(){
            _boatObject.transform.position = position; 
            _boatObject.transform.rotation = QuaternionRot;
        }

        ~BoatScenarioRT() {
            ws.Close();
        }
    }
}