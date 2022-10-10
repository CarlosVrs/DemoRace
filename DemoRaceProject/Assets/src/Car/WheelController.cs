using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class WheelController : MonoBehaviour
{
    [Header("Suspension")]
    public float springStrength;
    public float springDamper;
    public float springRestDistance;
    public float springTravel;

    [Header("Wheel")]
    public WheelPos pos;
    public float wheelRadius;
    [Range(0, 1)]
    public float wheelGripFactor;
    [Range(0, 1)]
    public float frictionFactor;
    public float wheelMass;
    [HideInInspector]
    public float friction;
    
    private float springMaxLength;
    public Transform wheelTransform;
    public Transform wheelVisual;

    private Rigidbody carRb;
    private CarController cc; 
    private float offset;
    private RaycastHit raycastHit;
    private bool rayDidHit;
    private float carSpeed;
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

    void Start()
    {
        carRb = transform.root.GetComponent<Rigidbody>();
        cc = transform.root.GetComponent<CarController>();

        springStrength *= carRb.mass;
        springDamper *= carRb.mass / 100f;
        springMaxLength = springRestDistance + springTravel;

        friction = 0.01f;
    }
}
