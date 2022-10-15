using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class CarController : MonoBehaviour
{
    [Header("Physics Model")]
    public CarRC carModel;
    public Rigidbody rb;   

    [Header("Wheels")]
    public GameObject wheelModel;

    [Header("Inputs")]
    public float accelInput;
    public float steeringInput;
    public float resetInput;

    private PlayerInput playerInput;
    private InputActionMap inputActionMap;
    private WheelController[] wheels;
    private RaycastHit raycastHit;
    private bool rayDidHit;
    private float offset;
    private bool isGrounded;
    private float antiRollL;
    private float antiRollR;
    private float carSpeed; 
    private float wheelBase; 
    private float rearTrack;  
    private float akAngleL;
    private float akAngleR;
    private float normalizedSpeed;
    
    WheelController wheelSetup(WheelController.WheelPos pos){
        GameObject newWheel = Instantiate(wheelModel, transform, false);
        WheelController wheelC = newWheel.GetComponent<WheelController>(); 
        Transform[] visuals;
        switch (pos)
        {
            case WheelController.WheelPos.FRONT_RIGHT:
                wheelC.pos = WheelController.WheelPos.FRONT_RIGHT;
                newWheel.transform.localPosition += carModel.frontOffset;

                visuals = wheelC.wheelVisuals.GetComponentsInChildren<Transform>();
                for (int vis = 1; vis < visuals.Length; vis++)
                    visuals[vis].Rotate(Vector3.up * 180, Space.Self);

                break;

            case WheelController.WheelPos.FRONT_LEFT:
                wheelC.pos = WheelController.WheelPos.FRONT_LEFT;
                newWheel.transform.localPosition += carModel.frontOffset + new Vector3(-(carModel.frontOffset.x * 2f), 0, 0);
                
                break;

            case WheelController.WheelPos.REAR_RIGHT:
                wheelC.pos = WheelController.WheelPos.REAR_RIGHT;
                newWheel.transform.localPosition += carModel.rearOffset + new Vector3(0, 0, -(carModel.rearOffset.z * 2));
                
                visuals = wheelC.wheelVisuals.GetComponentsInChildren<Transform>();                
                for (int vis = 1; vis < visuals.Length; vis++)
                    visuals[vis].Rotate(Vector3.up * 180, Space.Self);
                    
                break;
                
            case WheelController.WheelPos.REAR_LEFT:
                wheelC.pos = WheelController.WheelPos.REAR_LEFT;
                newWheel.transform.localPosition += carModel.rearOffset + new Vector3(-(carModel.rearOffset.x * 2f), 0, -(carModel.rearOffset.z * 2)); 

                break;
        }
        
        return wheelC;
    }

    void Start()
    {
        wheels = new WheelController[4];
        
        if(wheelModel != null){
            wheels[0] = wheelSetup(WheelController.WheelPos.FRONT_LEFT);
            wheels[1] = wheelSetup(WheelController.WheelPos.FRONT_RIGHT);
            wheels[2] = wheelSetup(WheelController.WheelPos.REAR_LEFT);
            wheels[3] = wheelSetup(WheelController.WheelPos.REAR_RIGHT);
        }

        rb.mass = carModel.mass;
        rb.collisionDetectionMode = carModel.collisionDetectionMode;
        rb.inertiaTensor = carModel.InertiaTensor * carModel.mass;
        rb.inertiaTensorRotation = Quaternion.identity;
        rb.centerOfMass = carModel.CenterOfMass;

        wheelBase = carModel.frontOffset.z + carModel.rearOffset.z;
        rearTrack = carModel.rearOffset.x * 2;

        playerInput = GetComponent<PlayerInput>();
        inputActionMap = playerInput.actions.FindActionMap("Vehicle");
    }

    void FixedUpdate(){
        isGrounded = false;
        foreach (WheelController wheelC in wheels)
        {
            rayDidHit = Physics.Raycast(wheelC.springTransform.position, -wheelC.springTransform.up, out raycastHit, wheelC.model.springRestDistance);
            accelInput = inputActionMap.FindAction("Accelerate").ReadValue<float>() - inputActionMap.FindAction("Reverse/Brake").ReadValue<float>();
            isGrounded |= rayDidHit;

            if(rayDidHit){

                carSpeed = Vector3.Dot(wheelC.springTransform.forward, rb.velocity);

                // Suspension spring force
                Vector3 springDir = wheelC.springTransform.up;
                Vector3 tireWorldVel = rb.GetPointVelocity(wheelC.wheelTransform.position);
                
                offset = wheelC.model.springRestDistance - raycastHit.distance;

                float vel = Vector3.Dot(springDir, tireWorldVel);
                float force = (offset * wheelC.model.springStrength * carModel.mass) - (vel * ((wheelC.model.springDamper * carModel.mass) / 100f));

                rb.AddForceAtPosition(springDir * (force / Time.fixedDeltaTime), wheelC.wheelTransform.position);
            
                // Grip force
                Vector3 steeringDir = wheelC.wheelTransform.right;
                tireWorldVel = rb.GetPointVelocity(wheelC.wheelTransform.position);

                normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / (carModel.topSpeed / CarRC.KMH_TO_MS));
                float availableGrip = wheelC.model.GripCurve.Evaluate(normalizedSpeed);
                
                float steeringVel = Vector3.Dot(steeringDir, tireWorldVel);
                float desiredVelChange = -steeringVel * availableGrip;
                float desiredAccel = desiredVelChange / Time.fixedDeltaTime;

                rb.AddForceAtPosition(steeringDir * wheelC.model.wheelMass * desiredAccel, wheelC.wheelTransform.position);

                // Anti roll bars
                float availableAntirrol = carModel.antiRollCurve.Evaluate(normalizedSpeed);
                // Front
                if(wheelC.pos == WheelController.WheelPos.FRONT_LEFT)
                    antiRollL = (-wheelC.springTransform.InverseTransformPoint(raycastHit.point).y ) / wheelC.model.springRestDistance;
                
                if(wheelC.pos == WheelController.WheelPos.FRONT_RIGHT)
                    antiRollR = (-wheelC.springTransform.InverseTransformPoint(raycastHit.point).y ) / wheelC.model.springRestDistance;
                
                float antiRollFrontForce = (antiRollL - antiRollR) * availableAntirrol * -carModel.antiRoll * rb.mass;
                
                if(wheelC.pos == WheelController.WheelPos.FRONT_RIGHT)
                    rb.AddForceAtPosition(springDir * -antiRollFrontForce, wheelC.springTransform.position);

                if( wheelC.pos == WheelController.WheelPos.FRONT_LEFT)
                    rb.AddForceAtPosition(springDir * antiRollFrontForce, wheelC.springTransform.position);
                
                // Rear
                if(wheelC.pos == WheelController.WheelPos.REAR_LEFT)
                    antiRollL = (-wheelC.springTransform.InverseTransformPoint(raycastHit.point).y ) / wheelC.model.springRestDistance;
                
                if(wheelC.pos == WheelController.WheelPos.REAR_RIGHT)
                    antiRollR = (-wheelC.springTransform.InverseTransformPoint(raycastHit.point).y ) / wheelC.model.springRestDistance;
                
                float antiRollRearForce = (antiRollL - antiRollR) * availableAntirrol * -carModel.antiRoll * rb.mass;
                
                if(wheelC.pos == WheelController.WheelPos.REAR_RIGHT)
                    rb.AddForceAtPosition(springDir * -antiRollRearForce, wheelC.springTransform.position);

                if( wheelC.pos == WheelController.WheelPos.REAR_LEFT)
                    rb.AddForceAtPosition(springDir * antiRollRearForce, wheelC.springTransform.position);
                    
                // Friction
                if (!(carSpeed < 0.05 && -0.05 < carSpeed)){
                    float desiredFrictionChange = Mathf.Sign(carSpeed) * -wheelC.model.frictionFactor * wheelC.model.friction;
                    float desiredFrictionAccel = desiredFrictionChange / Time.fixedDeltaTime;

                    rb.AddForceAtPosition(wheelC.wheelTransform.forward * rb.mass * desiredFrictionAccel, wheelC.wheelTransform.position);

                }
                
                // Acceleration 
                Vector3 accelDir = wheelC.wheelTransform.forward;

                if(accelInput > 0.0f && carSpeed < carModel.topSpeed){
                    float availableTorque = carModel.torqueCurve.Evaluate(normalizedSpeed) * accelInput;

                    rb.AddForceAtPosition(accelDir * availableTorque * carModel.torque, wheelC.wheelTransform.position, ForceMode.Impulse);
                    
                }
                
                // Braking and reverse
                if(accelInput < 0.0f && carSpeed > -carModel.reverseTopSpeed){
                    // Braking
                    float availableTorque = carModel.brakeCurve.Evaluate(normalizedSpeed) * accelInput;
                    availableTorque *= carModel.brake;     
                    
                    // Reverse
                    if(carSpeed <= 0.0f){
                        float normalizedReverseSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / (carModel.reverseTopSpeed / CarRC.KMH_TO_MS));
                        availableTorque = carModel.torqueCurve.Evaluate(normalizedReverseSpeed) * accelInput;
                        availableTorque *= carModel.torque;

                    }

                    rb.AddForceAtPosition(accelDir * availableTorque, wheelC.wheelTransform.position, ForceMode.Impulse);
                    
                }
            }
        }

        rb.centerOfMass = isGrounded ? carModel.CenterOfMass : carModel.LowerCenterOfMass;
    }

    void Update(){
        steeringInput = inputActionMap.FindAction("Steering").ReadValue<float>();
        resetInput = inputActionMap.FindAction("Reset").ReadValue<float>();
        
        // Steering
        akAngleL = akAngleR = 0;
        float availableTurnRadius = carModel.turnRadiusCurve.Evaluate(normalizedSpeed);
        // Steering Right
        if(steeringInput > 0){
            akAngleL = Mathf.Rad2Deg * Mathf.Atan(wheelBase / ((availableTurnRadius * carModel.turnRadius) + (rearTrack / 2))) * steeringInput;
            akAngleR = Mathf.Rad2Deg * Mathf.Atan(wheelBase / ((availableTurnRadius * carModel.turnRadius) - (rearTrack / 2))) * steeringInput;
        }
        // Steering Left
        if(steeringInput < 0){
            akAngleL = Mathf.Rad2Deg * Mathf.Atan(wheelBase / ((availableTurnRadius * carModel.turnRadius) - (rearTrack / 2))) * steeringInput;
            akAngleR = Mathf.Rad2Deg * Mathf.Atan(wheelBase / ((availableTurnRadius * carModel.turnRadius) + (rearTrack / 2))) * steeringInput;
        }

        foreach (WheelController wheelC in wheels)
        {
            rayDidHit = Physics.Raycast(wheelC.springTransform.position, -wheelC.springTransform.up, out raycastHit, wheelC.model.springRestDistance);
            
            // Akermann steering
            if(wheelC.pos == WheelController.WheelPos.FRONT_LEFT){
                wheelC.wheelTransform.localRotation = Quaternion.Euler(
                    wheelC.wheelTransform.localRotation.x, 
                    wheelC.wheelTransform.localRotation.y + akAngleL, 
                    wheelC.wheelTransform.localRotation.z
                );
            }

            if(wheelC.pos == WheelController.WheelPos.FRONT_RIGHT){
                wheelC.wheelTransform.localRotation = Quaternion.Euler(
                    wheelC.wheelTransform.localRotation.x, 
                    wheelC.wheelTransform.localRotation.y + akAngleR, 
                    wheelC.wheelTransform.localRotation.z
                );
            }

            // Visual suspension
            if(rayDidHit){
                wheelC.wheelTransform.localPosition = new Vector3(
                    wheelC.wheelTransform.localPosition.x, 
                    wheelC.model.wheelRadius - raycastHit.distance, 
                    wheelC.wheelTransform.localPosition.z
                );

            }else{
                wheelC.wheelTransform.localPosition = new Vector3(
                    wheelC.wheelTransform.localPosition.x, 
                    wheelC.model.wheelRadius - wheelC.model.springRestDistance, 
                    wheelC.wheelTransform.localPosition.z
                );

            }

            // Visual spin speed
            wheelC.spinSpeedFrame = (wheelC.springTransform.position - wheelC.posInLastFrame).magnitude;
            wheelC.posInLastFrame = wheelC.springTransform.position;
            
            if(!(accelInput < 0 && carSpeed > 0)){
                float direction = accelInput == 0 ? Mathf.Sign(carSpeed) : Mathf.Sign(accelInput);
                float valueToRotate = wheelC.spinSpeedFrame / (2 * Mathf.PI * wheelC.model.wheelRadius * direction);
                wheelC.wheelVisuals.Rotate(Vector3.right, valueToRotate * 360, Space.Self);

            }
        }

        if(resetInput == 1){
            transform.rotation = Quaternion.identity;
            transform.position += Vector3.up;
        }
    }
}
