using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlayerController : MonoBehaviour
{
    [Header("References")]
    public Transform playerTransorm;
    public Transform camPosTarget;
    public CarController carController;

    [Header("Camera Settings")]
    [Range(100,1000)]
    public int rotationSpeed = 600;
    public Vector3 offsets;

    [Header("Inputs")]
    [Range(-1,1)]
    public float sideInput;
    void Start()
    {
        if(playerTransorm == null)
            playerTransorm = gameObject.GetComponent<Transform>();

        if(camPosTarget == null)
            camPosTarget = transform.Find("CamPosTarget");

        if(carController == null)
            carController = gameObject.GetComponent<CarController>();
    }

    void LateUpdate(){
        camPosTarget.localPosition += offsets; 
        camPosTarget.RotateAround(playerTransorm.position, Vector3.up, sideInput * Time.deltaTime * rotationSpeed);
    }
}
