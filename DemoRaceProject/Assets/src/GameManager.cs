using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GameManager : MonoBehaviour
{
    [Header("FrameRate")]
    public int targetFrameRate = 60;
    
    [Header("Player")]
    public PlayerController playerController;

    void Start()
    {
        if(playerController == null)
            playerController = FindObjectOfType<PlayerController>();
        
        QualitySettings.vSyncCount = 0;
        Application.targetFrameRate = targetFrameRate;
    }
}
