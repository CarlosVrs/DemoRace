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

    private float friction;
    
    private float springMaxLength;
    public Transform wheelTransform;
    public Transform wheelVisual;

    private Rigidbody carRb;
    private CarController cc; 
    private float offset;
    private RaycastHit raycastHit;
    private bool rayDidHit;
    private float carSpeed;

    private float spinSpeedFrame;
    private Vector3 posInLastFrame;

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

    void FixedUpdate(){
        rayDidHit = Physics.Raycast(transform.position, -transform.up, out raycastHit, springMaxLength);
        
        if(rayDidHit){
            //Suspension spring force
            Vector3 springDir = wheelTransform.up;
            Vector3 tireWorldVel = carRb.GetPointVelocity(wheelTransform.position) / Time.fixedDeltaTime;
            
            offset = springRestDistance - raycastHit.distance;

            float vel = Vector3.Dot(springDir, tireWorldVel);

            float force = (offset * springStrength) - (vel * springDamper);

            carRb.AddForceAtPosition(springDir * force, wheelTransform.position);

            // Debug.DrawLine(transform.position,  new Vector3(transform.position.x, offset, transform.position.z), Color.blue);
            // Debug.DrawLine(new Vector3(transform.position.x, offset, transform.position.z), raycastHit.point, Color.green);
        
            //Steering force
            Vector3 steeringDir = wheelTransform.right;
            tireWorldVel = carRb.GetPointVelocity(wheelTransform.position);

            float steeringVel = Vector3.Dot(steeringDir, tireWorldVel);
            float desiredVelChange = -steeringVel * wheelGripFactor;
            float desiredAccel = desiredVelChange / Time.fixedDeltaTime;

            carRb.AddForceAtPosition(steeringDir * wheelMass * desiredAccel, wheelTransform.position);

            //Acceleration 
            Vector3 accelDir = wheelTransform.forward;
            carSpeed = Vector3.Dot(transform.forward, carRb.velocity);

            if(cc.accelInput > 0.0f && carSpeed < cc.carTopSpeed){
                float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / cc.carTopSpeed);
                float availableTorque = cc.torqueCurve.Evaluate(normalizedSpeed) * cc.accelInput;

                carRb.AddForceAtPosition(accelDir * availableTorque * cc.torque, wheelTransform.position, ForceMode.Impulse);
                
            }
            
            // Braking and reverse
            if(cc.accelInput < 0.0f && carSpeed > -cc.carBackwardsTopSpeed){
                // Braking
                float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / cc.carTopSpeed);
                float availableTorque = cc.brakeCurve.Evaluate(normalizedSpeed) * cc.accelInput;
                availableTorque *= cc.brakeForze;     
                
                //Reverse
                if(carSpeed <= 0.0f){
                    normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / cc.carBackwardsTopSpeed);
                    availableTorque = cc.torqueCurve.Evaluate(normalizedSpeed) * cc.accelInput;
                    availableTorque *= cc.torque;

                }

                carRb.AddForceAtPosition(accelDir * availableTorque, wheelTransform.position, ForceMode.Impulse);
                
            }
            
            //Friction
            if (!(carSpeed < 0.01 && -0.01 < carSpeed)){
                float desiredFrictionChange = Mathf.Sign(carSpeed) * -frictionFactor * friction;
                float desiredFrictionAccel = desiredFrictionChange / Time.fixedDeltaTime;

                carRb.AddForceAtPosition(accelDir * carRb.mass * desiredFrictionAccel, wheelTransform.position);

            }

        }

        //Visual spin speed
        spinSpeedFrame = (transform.position - posInLastFrame).magnitude;
        posInLastFrame = transform.position;
    }

    void Update(){

        //Visual suspension
        if(rayDidHit){
            wheelVisual.transform.localPosition = new Vector3(
                wheelVisual.transform.localPosition.x , 
                wheelRadius - raycastHit.distance, 
                wheelVisual.transform.localPosition.z
            );

        }else{
            wheelVisual.transform.localPosition = new Vector3(
                wheelVisual.transform.localPosition.x , 
                wheelRadius - springMaxLength, 
                wheelVisual.transform.localPosition.z
            );

        }

        // Visual rotation
        if(!(cc.accelInput < 0 && carSpeed > 0)){
            float direction = cc.accelInput == 0 ? Mathf.Sign(carSpeed) : Mathf.Sign(cc.accelInput);
            float valueToRotate = spinSpeedFrame / (2 * Mathf.PI * wheelRadius) * direction;
            wheelVisual.Rotate(Vector3.right, valueToRotate * 360, Space.Self);

        }
        
    }
}
