using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GameManager : MonoBehaviour
{
    [Header("Player")]
    public PlayerController playerController;

    void Start()
    {
        if(playerController == null)
            playerController = FindObjectOfType<PlayerController>();
    }
}
