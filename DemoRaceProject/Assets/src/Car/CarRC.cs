using UnityEngine;

[CreateAssetMenu(fileName ="New Car", menuName = "CarRC")]
public class CarRC : ScriptableObject
{
    [Header("Physics Model\n")]
    public float mass = 1000;
    public CollisionDetectionMode collisionDetectionMode;
    
    // <-- Calculated with Box Collider on car RB
    public Vector3 InertiaTensor;
    public Vector3 CenterOfMass;
    public Vector3 LowerCenterOfMass;
    // -->

    [Header("Car Specs\n")]
    [Tooltip("In m/s")]
    [Range(15, SPEED_LIMIT)]
    public float topSpeed;
    [Range(1, REVERSE_SPEED_LIMIT)]
    public float reverseTopSpeed;
    [Range(1, 200)]
    public float torque;
    public AnimationCurve torqueCurve;
    [Range(100,400)]
    public float brake;
    public AnimationCurve brakeCurve;
    [Range(1, 200)]
    public float antiRoll;
    public AnimationCurve antiRollCurve;
    
    [Header("AkermannSteering\n")]
    public Vector3 frontOffset;
    public Vector3 rearOffset;
    [Range(1,60)]
    public float turnRadius;
    public AnimationCurve turnRadiusCurve;

    // Max configurable speeds in km/s: 1 m/s -> 3.6 km/h
    // 1 Unity unit -> 1 m 
    public const float SPEED_LIMIT = 288;
    public const float REVERSE_SPEED_LIMIT = 54;
    public const float KMH_TO_MS = 3.6f;
}
