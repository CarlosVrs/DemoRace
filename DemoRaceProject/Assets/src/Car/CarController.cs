using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

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

    private float carSpeed;    

    [Header("Wheels")]
    public GameObject wheelModel;
    public Vector3 frontOffset;
    public Vector3 rearOffset;

    [Header("Inputs")]
    public float accelInput;
    PlayerInput playerInput;
    InputActionMap inputActionMap;
    
    private WheelController[] wheels;
    private RaycastHit raycastHit;
    private bool rayDidHit;
    private float offset;

    
    // Start is called before the first frame update
    void Start()
    {
        wheels = new WheelController[4];
        
        if(wheelModel != null){
            wheels[0] = wheelSetup(WheelController.WheelPos.FRONT_LEFT);
            wheels[1] = wheelSetup(WheelController.WheelPos.FRONT_RIGHT);
            wheels[2] = wheelSetup(WheelController.WheelPos.REAR_LEFT);
            wheels[3] = wheelSetup(WheelController.WheelPos.REAR_RIGHT);
        }

        rb.mass = mass;
        rb.collisionDetectionMode = detectionMode;

        playerInput = GetComponent<PlayerInput>();
        inputActionMap = playerInput.actions.FindActionMap("Vehicle");
    }

    WheelController wheelSetup(WheelController.WheelPos pos){
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

        return wheelController;
    }

    void FixedUpdate(){
        foreach (WheelController wheelC in wheels)
        {
            rayDidHit = Physics.Raycast(wheelC.transform.position, -wheelC.transform.up, out raycastHit, wheelC.springRestDistance);

            if(rayDidHit){
                //Suspension spring force
                Vector3 springDir = wheelC.wheelTransform.up;
                Vector3 tireWorldVel = rb.GetPointVelocity(wheelC.wheelTransform.position) / Time.fixedDeltaTime;
                
                offset = wheelC.springRestDistance - raycastHit.distance;

                float vel = Vector3.Dot(springDir, tireWorldVel);

                float force = (offset * wheelC.springStrength) - (vel * wheelC.springDamper);

                rb.AddForceAtPosition(springDir * force, wheelC.wheelTransform.position);
            
                //Steering force
                Vector3 steeringDir = wheelC.wheelTransform.right;
                tireWorldVel = rb.GetPointVelocity(wheelC.wheelTransform.position);

                float steeringVel = Vector3.Dot(steeringDir, tireWorldVel);
                float desiredVelChange = -steeringVel * wheelC.wheelGripFactor;
                float desiredAccel = desiredVelChange / Time.fixedDeltaTime;

                rb.AddForceAtPosition(steeringDir * wheelC.wheelMass * desiredAccel, wheelC.wheelTransform.position);
            
                //Friction
                if (!(carSpeed < 0.01 && -0.01 < carSpeed)){
                    float desiredFrictionChange = Mathf.Sign(carSpeed) * -wheelC.frictionFactor * wheelC.friction;
                    float desiredFrictionAccel = desiredFrictionChange / Time.fixedDeltaTime;

                    rb.AddForceAtPosition(wheelC.wheelTransform.forward * rb.mass * desiredFrictionAccel, wheelC.wheelTransform.position);

                }

                //Acceleration 
                Vector3 accelDir = wheelC.wheelTransform.forward;
                carSpeed = Vector3.Dot(wheelC.transform.forward, rb.velocity);

                accelInput = inputActionMap.FindAction("Accelerate").ReadValue<float>() - inputActionMap.FindAction("Reverse/Brake").ReadValue<float>();

                if(accelInput > 0.0f && carSpeed < carTopSpeed){
                    float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / carTopSpeed);
                    float availableTorque = torqueCurve.Evaluate(normalizedSpeed) * accelInput;

                    rb.AddForceAtPosition(accelDir * availableTorque * torque, wheelC.wheelTransform.position, ForceMode.Impulse);
                    
                }
                
                // Braking and reverse
                if(accelInput < 0.0f && carSpeed > -carBackwardsTopSpeed){
                    // Braking
                    float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / carTopSpeed);
                    float availableTorque = brakeCurve.Evaluate(normalizedSpeed) * accelInput;
                    availableTorque *= brakeForze;     
                    
                    //Reverse
                    if(carSpeed <= 0.0f){
                        normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / carBackwardsTopSpeed);
                        availableTorque = torqueCurve.Evaluate(normalizedSpeed) * accelInput;
                        availableTorque *= torque;

                    }

                    rb.AddForceAtPosition(accelDir * availableTorque, wheelC.wheelTransform.position, ForceMode.Impulse);
                    
                }
            }

            
        }
    }

    void Update(){
        foreach (WheelController wheelC in wheels)
        {
            rayDidHit = Physics.Raycast(wheelC.transform.position, -wheelC.transform.up, out raycastHit, wheelC.springRestDistance);
            //Visual suspension
            if(rayDidHit){
                wheelC.wheelVisual.transform.localPosition = new Vector3(
                    wheelC.wheelVisual.transform.localPosition.x, 
                    wheelC.wheelRadius - raycastHit.distance, 
                    wheelC.wheelVisual.transform.localPosition.z
                );

            }else{
                wheelC.wheelVisual.transform.localPosition = new Vector3(
                    wheelC.wheelVisual.transform.localPosition.x, 
                    wheelC.wheelRadius - wheelC.springRestDistance, 
                    wheelC.wheelVisual.transform.localPosition.z
                );

            }

            //Visual spin speed
            wheelC.spinSpeedFrame = (wheelC.transform.position - wheelC.posInLastFrame).magnitude;
            wheelC.posInLastFrame = wheelC.transform.position;
            
            if(!(accelInput < 0 && carSpeed > 0)){
                float direction = accelInput == 0 ? Mathf.Sign(carSpeed) : Mathf.Sign(accelInput);
                float valueToRotate = wheelC.spinSpeedFrame / (2 * Mathf.PI * wheelC.wheelRadius * direction);
                wheelC.wheelVisual.Rotate(Vector3.right, valueToRotate * 360, Space.Self);

            }
        }
    }
}
