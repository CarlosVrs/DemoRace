using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    [Header("References")]
    public GameManager gameManager;
    public Transform playerTransorm;
    public Transform camPosTarget;

    [Header("Settings")]
    [Range(0.1f,200)]
    public float lerpSpeed = 1;
    [Range(-2, 2)]
    public float LookAtAngle = 0;
    public Vector3 offsets;
    Vector3 lookAtOffset;

    
    void Start()
    {
        if(gameManager == null)
            gameManager = FindObjectOfType<GameManager>();
        
        if(playerTransorm == null)
            playerTransorm = gameManager.playerController.playerTransorm;
        
        if(camPosTarget == null)
            camPosTarget = gameManager.playerController.camPosTarget;

    }

    void LateUpdate(){
        lookAtOffset = playerTransorm.position;
        lookAtOffset.y += LookAtAngle;
        transform.position = camPosTarget.position + offsets;
        transform.LookAt(lookAtOffset);
    }
}
