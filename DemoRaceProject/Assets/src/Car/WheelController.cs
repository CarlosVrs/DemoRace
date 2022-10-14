using UnityEngine;

public class WheelController : MonoBehaviour
{
    [Header("References")]
    public WheelRC model;
    public Transform springTransform;
    public Transform wheelTransform;
    public Transform wheelVisuals;

    [Header("Wheel")]
    public WheelPos pos;
    [HideInInspector]
    public float spinSpeedFrame;
    [HideInInspector]
    public Vector3 posInLastFrame;

    public enum WheelPos{
        FRONT_LEFT,
        FRONT_RIGHT,
        REAR_LEFT,
        REAR_RIGHT
    }
}
