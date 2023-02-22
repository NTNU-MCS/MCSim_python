using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using UnityEditor;

namespace Gemini.EMRS.ScenarioGenerator
{
    public class AisBoat
    {
        public GameObject boatObject;
        public string id;
        public Vector3 pos = new Vector3(0f, 0f, 0f);
        public Quaternion rot = Quaternion.Euler(new Vector3(0f, 0f, 0f));

        public AisBoat(string id)
        { 
            this.id = id;
        }
    }
}