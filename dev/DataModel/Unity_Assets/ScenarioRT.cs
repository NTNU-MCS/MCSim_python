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
        private BoatScenarioRT _boatScenario;

        private double sim_time = -1d; 

        void Start()
        {
            SetupBoat(); 
        }
        
        void FixedUpdate() { 
                _boatScenario.UpdateVessel();
                sim_time = _boatScenario.time;
        }

    void OnDisable()
    {   
        _boatScenario.running = false;
        
    }

        private void SetupBoat()
        {
            BoatPrefab = Instantiate(BoatPrefab, new Vector3(0, 0, 0), Quaternion.identity);
            _boatScenario = new BoatScenarioRT(BoatPrefab);
        }
    }
}