using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class CameraFollow : MonoBehaviour
{
    [Header("References")]
    public Transform playerTransorm;
    public Transform camTransform;

    [Header("Settings")]
    [Range(-2, 2)]
    public float LookAtAngle;
    [Range(0f, 1f)]
    public float rotationSpeed;
    public Vector3 offsets;

    [Header("Inputs")]
    [Range(-1,1)]
    public float sideInput;
    PlayerInput playerInput;
    InputActionMap inputActionMap;
    Vector3 lookAtOffset;

    
    void Start()
    {
        playerInput = playerTransorm.GetComponent<PlayerInput>();
        inputActionMap = playerInput.actions.FindActionMap("Vehicle");

        
    }

    void LateUpdate(){
        sideInput = inputActionMap.FindAction("RotateCam").ReadValue<float>();

        lookAtOffset = playerTransorm.position;
        lookAtOffset.y += LookAtAngle;
        transform.position = Vector3.MoveTowards(transform.position, playerTransorm.position, rotationSpeed / Time.deltaTime);
        transform.Rotate(Vector3.up, sideInput * (rotationSpeed / Time.deltaTime), Space.Self);

        camTransform.LookAt(lookAtOffset, Vector3.up);
        camTransform.localPosition = offsets;
    }
}
