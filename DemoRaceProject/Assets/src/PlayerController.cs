using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

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
    PlayerInput playerInput;
    InputActionMap inputActionMap;
    void Start()
    {
        if(playerTransorm == null)
            playerTransorm = gameObject.GetComponent<Transform>();

        if(camPosTarget == null)
            camPosTarget = transform.Find("CamPosTarget");

        if(carController == null)
            carController = gameObject.GetComponent<CarController>();

        playerInput = GetComponent<PlayerInput>();
        inputActionMap = playerInput.actions.FindActionMap("Vehicle");
    }

    void LateUpdate(){
        sideInput = inputActionMap.FindAction("RotateCam").ReadValue<float>();

        camPosTarget.localPosition += offsets; 
        camPosTarget.RotateAround(playerTransorm.position, playerTransorm.up, sideInput * Time.deltaTime * rotationSpeed);
    }
}
