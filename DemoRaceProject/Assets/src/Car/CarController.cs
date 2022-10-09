using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarController : MonoBehaviour
{
    [Header("Physics Model")]
    public Rigidbody rb;
    public BoxCollider col;
    public float mass = 1000;
    public CollisionDetectionMode detectionMode;

    [Header("Car Specs")]
    public float carTopSpeed;
    public float carBackwardsTopSpeed;
    public float torque;
    public float brakeForze;
    public AnimationCurve torqueCurve;
    public AnimationCurve brakeCurve;    

    [Header("Wheels")]
    public GameObject wheelModel;
    public Vector3 frontOffset;
    public Vector3 rearOffset;

    [Header("Inputs")]
    [Range(-1, 1)]
    public float accelInput;
    
    private GameObject[] wheels;

    
    // Start is called before the first frame update
    void Start()
    {
        wheels = new GameObject[4];
        
        if(wheelModel != null){
            wheels[0] = wheelSetup(WheelController.WheelPos.FRONT_LEFT);
            wheels[1] = wheelSetup(WheelController.WheelPos.FRONT_RIGHT);
            wheels[2] = wheelSetup(WheelController.WheelPos.REAR_LEFT);
            wheels[3] = wheelSetup(WheelController.WheelPos.REAR_RIGHT);
        }

        rb.mass = mass;
        rb.collisionDetectionMode = detectionMode; 
    }

    GameObject wheelSetup(WheelController.WheelPos pos){
        GameObject newWheel = Instantiate(wheelModel, transform, false);
        WheelController wheelController = newWheel.GetComponent<WheelController>(); 
        switch (pos)
        {
            case WheelController.WheelPos.FRONT_RIGHT:
                wheelController.pos = WheelController.WheelPos.FRONT_RIGHT;
                newWheel.transform.localPosition += frontOffset; 
                break;
            case WheelController.WheelPos.FRONT_LEFT:
                wheelController.pos = WheelController.WheelPos.FRONT_LEFT;
                newWheel.transform.localPosition += frontOffset + new Vector3(-(frontOffset.x * 2f), 0, 0);
                break;
            case WheelController.WheelPos.REAR_RIGHT:
                wheelController.pos = WheelController.WheelPos.REAR_RIGHT;
                newWheel.transform.localPosition += rearOffset + new Vector3(0, 0, -(rearOffset.z * 2)); 
                break;
            case WheelController.WheelPos.REAR_LEFT:
                wheelController.pos = WheelController.WheelPos.REAR_LEFT;
                newWheel.transform.localPosition += rearOffset + new Vector3(-(rearOffset.x * 2f), 0, -(rearOffset.z * 2)); 
                break;
        }

        return newWheel;
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
