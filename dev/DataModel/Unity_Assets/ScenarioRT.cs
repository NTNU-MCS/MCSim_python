using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Gemini.EMRS.Core;
using System.Threading;
using System.Threading.Tasks;
using System;

namespace Gemini.EMRS.ScenarioGenerator {
    public class ScenarioRT: MonoBehaviour
    {
        public GameObject BoatPrefab;
        public GameObject aisBoatPrefab;
        public float xRotOffset = 0;
        public float yRotOffset = 0;
        public float zRotOffset = 0;
        private BoatScenarioRT _boatScenario;

        private double sim_time = -1d; 

        void Start()
        {
            SetupBoat(); 
        }
        
        void FixedUpdate() { 
                _boatScenario.UpdateVessel();
            UpdateAisVessels();
            sim_time = _boatScenario.time;
        }

        void OnDisable()
        {   
            _boatScenario.running = false;
            
        }

        void UpdateAisVessels()
        { 
            foreach (AisBoat item in _boatScenario.aisObjects)
            {
                if (item.boatObject == null) { 
                    item.boatObject = Instantiate(
                        aisBoatPrefab, 
                        new Vector3(0, 0, 0), 
                        Quaternion.identity
                        );
                }  
                
                item.boatObject.transform.position = item.pos; 
                item.boatObject.transform.rotation = item.rot;
            }             
        }

        private void SetupBoat()
        {
            // BoatPrefab = Instantiate(BoatPrefab, new Vector3(0, 0, 0), Quaternion.identity);
            _boatScenario = new BoatScenarioRT(BoatPrefab, xRotOffset, yRotOffset, zRotOffset);
        }
    }
}