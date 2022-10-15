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
    [Range(15,200)]
    public float topSpeed;
    [Range(1,20)]
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
    
}
