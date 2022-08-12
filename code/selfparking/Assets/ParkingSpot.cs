using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ParkingSpot : MonoBehaviour
{
    private Material material;
    private Color green = new Color(0.29412f, 0.70980f, 0.26275f);
    private Color red = new Color(1f, 0f, 0f);
    private void OnTriggerStay(Collider other)
    {
        if (other.tag == "agent")
        {
            MeshRenderer meshRenderer = GetComponent<MeshRenderer>();
            // Debug.Log(meshRenderer.material);
            material = meshRenderer.material;
            material.SetColor("_Color", green);
        }
    }

    private void OnTriggerExit(Collider other)
    {
        MeshRenderer meshRenderer = GetComponent<MeshRenderer>();
        // Debug.Log(meshRenderer.material);
        material = meshRenderer.material;
        material.SetColor("_Color", red);
    }
}
