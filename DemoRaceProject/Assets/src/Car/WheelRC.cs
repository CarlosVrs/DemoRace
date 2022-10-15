using UnityEngine;

[CreateAssetMenu(fileName = "New wheel", menuName = "WheelRC")]
public class WheelRC : ScriptableObject
{
    [Header("Suspension")]
    public float springStrength;
    public float springDamper;
    [Range(0, 5)]
    public float springRestDistance;

    [Header("Wheel")]
    [Range(1,200)]
    public float wheelMass;
    [Range(0.1f, 2)]
    public float wheelRadius;
    public AnimationCurve GripCurve;
    [Range(0.01f, 0.1f)]
    public float friction = 0.01f;
    [Range(0, 1)]
    public float frictionFactor;
    
    
    
}
