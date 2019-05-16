using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using VRTK;

public class Throwing : MonoBehaviour
{
    public enum HammerState
    {
        released,
        grabbed,
        flying,
        returning
    }

    public HammerState hammerState = HammerState.released;

    Rigidbody m_rigidbody;

    public VRTK_InteractableObject linkedObject;

    Vector3 m_acceleration = Vector3.zero;
    Vector3 lastVelocity = Vector3.zero;
    Vector3 throwAcceleration = Vector3.zero;
    Vector3 throwVelocity = Vector3.zero;
    [SerializeField] double timeBetweenAccelerationMesurements = 0.1f;
    double timeSinceLastAccelerationMesurement = 0f;

    [SerializeField] double minimumSpeedToFlyingThrow = 1f;

    [Space]

    [SerializeField] [Range(0.1f, 10f)] double velocityMultiplier = 1f;
    [SerializeField] [Range(0f, 10f)] double accelerationMultiplier = 1f;

    [Space]

    [SerializeField] [Range(0f, 1f)] double interpolateRotationFactor = 0.1f;
    double interpolateRawValue = 0.1f;

    [Space]
    [Header("Controllers")]
    [SerializeField] VRTK_ControllerEvents leftControllerEvents;
    [SerializeField] VRTK_ControllerEvents rightControllerEvents;

    [SerializeField] VRTK_ObjectAutoGrab leftAutoGrab;
    [SerializeField] VRTK_ObjectAutoGrab rightAutoGrab;

    int countToDisableAutoGrab = 0;
    [SerializeField] int maxCountToDisableAutoGrab = 10;

    Transform hmdTransform;
    [SerializeField] GameObject leftController;
    [SerializeField] GameObject rightController;

    bool leftControllerCallHammerButtonPreviousState = false;
    bool rightControllerCallHammerButtonPreviousState = false;

    [Space]
    [Header("Bezier Curve")]
    Vector3 curvePoint = Vector3.zero;
    [SerializeField] GameObject curvePointGameObject;

    [Space]
    [SerializeField] Vector3 leftHandCurvePointOffset = Vector3.right + Vector3.forward;
    [SerializeField] Vector3 rightHandCurvePointOffset = Vector3.left + Vector3.forward;
    [SerializeField] bool useRightHandOffSetForLeftHand = true;
    [SerializeField] float curvePointMinimumHeight = -0.5f;

    bool controllerToReturnIsRightHand = true;
    Vector3 originalPosition = Vector3.zero;
    Vector3 finalPosition = Vector3.zero;

    bool controllerCallHammerPreviousStateClicked = false;

    [SerializeField] float distanceToLift = 0.1f;
    [SerializeField] float liftingSpeed = 0.05f;
    float currentLiftedHeight = 0f;
    Vector3 initialPositionToLiftFrom = Vector3.zero;
    bool shouldLiftFromFloor = false;
    [SerializeField] float minimumSpeedToBeConsideredStopped = 0.05f;

    [Header("Parameters to call hammer back")]
    [SerializeField] [Range(0.01f, 10f)] public double bezierSpeedFactor = 1f;
    [Range(0f, 1f)] double bezierParameter = 0f;

    [SerializeField] bool automaticallyCalculateBezierReferencePointDistance = true;
    [SerializeField] float maximumSpeedWhenReturningWhenNotUsingBezierCurve = 20f;
    [SerializeField] float maximumForceIntensityOnReturning = 100f;
    [SerializeField] [Range(0.01f, 5f)] float distanceToConsiderHammerReturnedWhenNotUsingBezierCurve = 0.2f;

    bool shouldUseBezierCurve = true;
    [SerializeField] bool forceUseBezierCurve = true;

    protected virtual void OnEnable()
    {
        linkedObject = (linkedObject == null ? GetComponent<VRTK_InteractableObject>() : linkedObject);

        if (linkedObject != null)
        {
            linkedObject.InteractableObjectGrabbed += InteractableObjectGrabbed;
            linkedObject.InteractableObjectUngrabbed += InteractableObjectUngrabbed;
        }
    }

    protected virtual void OnDisable()
    {
        if (linkedObject != null)
        {
            linkedObject.InteractableObjectGrabbed -= InteractableObjectGrabbed;
            linkedObject.InteractableObjectUngrabbed += InteractableObjectUngrabbed;
        }
    }

    private void Awake()
    {
        m_rigidbody = GetComponent<Rigidbody>();

        
    }


    private void Update()
    {
        if (hmdTransform == null)
        {
            hmdTransform = VRTK_DeviceFinder.HeadsetTransform();
        }
        
        if(leftController == null)
        {
            leftController = VRTK_DeviceFinder.GetControllerLeftHand(true);
        }

        if(rightController == null)
        {
            rightController = VRTK_DeviceFinder.GetControllerRightHand(true);
        }


        /*if((!leftControllerEvents.gripClicked && leftControllerCallHammerButtonPreviousState) || (!rightControllerEvents.gripClicked && rightControllerCallHammerButtonPreviousState))
        {
            if(hammerState == HammerState.returning)
            {
                m_rigidbody.isKinematic = false;
                //shouldLiftFromFloor = false;
                if (m_rigidbody.velocity.magnitude >= minimumSpeedToFlyingThrow)
                {
                    Debug.Log("speed:" + m_rigidbody.velocity.magnitude.ToString());
                    hammerState = HammerState.flying;

                    //throwAcceleration = m_acceleration * (float)accelerationMultiplier;

                    //interpolateRawValue = 0.1f;
                }
                else
                {
                    hammerState = HammerState.released;
                    shouldUseBezierCurve = true;
                    m_rigidbody.velocity = Vector3.zero;
                    m_rigidbody.useGravity = true;
                }
            }
        }
        leftControllerCallHammerButtonPreviousState = leftControllerEvents.gripClicked;
        rightControllerCallHammerButtonPreviousState = rightControllerEvents.gripClicked;*/


        CheckIfCallingHammerBack();
        controllerCallHammerPreviousStateClicked = leftControllerEvents.gripClicked || rightControllerEvents.gripClicked;
        if (hammerState != HammerState.grabbed)
        {
            SetCurvePoint();
        }

        //if(leftAutoGrab.enabled || rightAutoGrab.enabled)
        //{
        //    countToDisableAutoGrab++;
        //}
        //if(countToDisableAutoGrab >= maxCountToDisableAutoGrab)
        //{
        //    countToDisableAutoGrab = 0;
        //    leftAutoGrab.enabled = false;
        //    rightAutoGrab.enabled = false;
        //}
    }

    private void FixedUpdate()
    {
        timeSinceLastAccelerationMesurement += Time.deltaTime;

        if (timeSinceLastAccelerationMesurement >= timeBetweenAccelerationMesurements)
        {
            timeSinceLastAccelerationMesurement = 0f;
            if (timeBetweenAccelerationMesurements != 0f)
            {
                m_acceleration = (m_rigidbody.velocity - lastVelocity) / (float)timeBetweenAccelerationMesurements;
            }
            lastVelocity = m_rigidbody.velocity;
        }

        if ((!leftControllerEvents.gripClicked && leftControllerCallHammerButtonPreviousState) || (!rightControllerEvents.gripClicked && rightControllerCallHammerButtonPreviousState))
        {
            Debug.Log("The speed:" + m_rigidbody.velocity.magnitude.ToString());
            if (hammerState == HammerState.returning)
            {
                Debug.Log("is returning");
                m_rigidbody.isKinematic = false;
                //shouldLiftFromFloor = false;
                Debug.Log("The speed::" + m_rigidbody.velocity.magnitude.ToString() + " minimumSpeedToFlyingThrow::" + minimumSpeedToFlyingThrow);
                if (m_rigidbody.velocity.magnitude >= minimumSpeedToFlyingThrow)
                {
                    hammerState = HammerState.flying;
                    shouldUseBezierCurve = false;

                    //throwAcceleration = m_acceleration * (float)accelerationMultiplier;

                    //interpolateRawValue = 0.1f;
                }
                else
                {
                    hammerState = HammerState.released;
                    shouldUseBezierCurve = true;
                    m_rigidbody.velocity = Vector3.zero;
                    m_rigidbody.angularVelocity = Vector3.zero;
                    m_rigidbody.useGravity = true;
                }
                Debug.Log("hammerState = " + hammerState);
            }
            Debug.Log("New speed:" + m_rigidbody.velocity.magnitude.ToString());
        }
        leftControllerCallHammerButtonPreviousState = leftControllerEvents.gripClicked;
        rightControllerCallHammerButtonPreviousState = rightControllerEvents.gripClicked;
        //Debug.Log("hammerState = " + hammerState);
        if (hammerState == HammerState.flying)
        {
            Debug.Log("HEATHROW");
            ThrowMovement();
        }

        if (hammerState == HammerState.returning)
        {
            ReturningMovement();
        }

    }

    private void InteractableObjectGrabbed(object sender, InteractableObjectEventArgs e)
    {
        //if(hammerState != HammerState.returning)
        //{
        //    hammerState = HammerState.grabbed;
        //}

        hammerState = HammerState.grabbed;
    }

    private void InteractableObjectUngrabbed(object sender, InteractableObjectEventArgs e)
    {
        hammerState = HammerState.released;
        shouldUseBezierCurve = true;

        throwVelocity = m_rigidbody.velocity * (float) velocityMultiplier;
        if(throwVelocity.magnitude >= minimumSpeedToFlyingThrow)
        {
            Debug.Log("ungrabbed");
            hammerState = HammerState.flying;
            shouldUseBezierCurve = false;
            m_rigidbody.useGravity = false;

            throwAcceleration = m_acceleration * (float)accelerationMultiplier;

            interpolateRawValue = 0.1f;
        }

        leftAutoGrab.enabled = false;
        rightAutoGrab.enabled = false;

        if(hammerState == HammerState.released)
        {
            m_rigidbody.useGravity = true;
        }

        
    }

    Vector3 deltaPosition = Vector3.zero;

    void ThrowMovement()
    {
        Debug.Log("ThrowMovement");
        Vector3 newPosition = transform.position + throwVelocity * (Time.fixedDeltaTime) + 0.5f * throwAcceleration * (Time.fixedDeltaTime) * (Time.fixedDeltaTime);
        throwVelocity += throwAcceleration * Time.fixedDeltaTime;
        deltaPosition = newPosition - transform.position;
        m_rigidbody.MovePosition(newPosition);
        //m_rigidbody.velocity = deltaPosition / Time.fixedDeltaTime;

        float angleToRotate = Vector3.Angle(transform.up, deltaPosition);
        Vector3 rotationAxis = Vector3.Cross(transform.up, deltaPosition);
        Quaternion rotation = Quaternion.AngleAxis(angleToRotate, rotationAxis);

        Vector3 rotatedAxisX = rotation * transform.right;
        Vector3 rotatedAxisZ = rotation * transform.forward;


        interpolateRawValue += Time.fixedDeltaTime;
        //m_rigidbody.MoveRotation(Quaternion.Lerp(transform.rotation, Quaternion.LookRotation(rotatedAxisZ, deltaPosition), (float)(interpolateRawValue * interpolateRotationFactor)));
        //m_rigidbody.MoveRotation(Quaternion.Lerp(transform.rotation, Quaternion.LookRotation(rotatedAxisZ, m_rigidbody.velocity), (float)(interpolateRawValue * interpolateRotationFactor)));
        //m_rigidbody.MoveRotation(Quaternion.LookRotation(rotatedAxisZ, deltaPosition));
        m_rigidbody.MoveRotation(Quaternion.LookRotation(rotatedAxisZ, m_rigidbody.velocity));
    }

    void ReturningMovement()
    {
        Debug.Log("ReturningMovement");
        if (currentLiftedHeight < distanceToLift && shouldLiftFromFloor)
        {
            m_rigidbody.isKinematic = true;
            currentLiftedHeight += liftingSpeed * Time.fixedDeltaTime;
            m_rigidbody.MovePosition(/*initialPositionToLiftFrom*/ transform.position + Vector3.up * liftingSpeed * Time.fixedDeltaTime);

            originalPosition = transform.position;
            return;
        }

        m_rigidbody.isKinematic = false;

        bezierParameter += Time.fixedDeltaTime * bezierSpeedFactor;

        if (forceUseBezierCurve)
        {
            shouldUseBezierCurve = true;
        }
        if (shouldUseBezierCurve)
        {
            Vector3 previousPosition = transform.position;
            Vector3 newPosition = GetBezierQuadraticCurvePoint((float)bezierParameter, originalPosition, curvePoint, finalPosition);
            deltaPosition = newPosition - transform.position;
            m_rigidbody.MovePosition(newPosition);
            Vector3 currentPosition = transform.position;
            Vector3 actualDeltaPosition = currentPosition - previousPosition;
            //m_rigidbody.velocity = deltaPosition / Time.fixedDeltaTime;

            float angleToRotate = Vector3.Angle(transform.up, deltaPosition);
            Vector3 rotationAxis = Vector3.Cross(transform.up, deltaPosition);
            Quaternion rotation = Quaternion.AngleAxis(angleToRotate, rotationAxis);

            Vector3 rotatedAxisX = rotation * transform.right;
            Vector3 rotatedAxisZ = rotation * transform.forward;


            interpolateRawValue += Time.fixedDeltaTime;
            m_rigidbody.MoveRotation(Quaternion.Lerp(transform.rotation, Quaternion.LookRotation(rotatedAxisZ, deltaPosition), (float)(interpolateRawValue * interpolateRotationFactor)));
            
            throwVelocity = deltaPosition / Time.fixedDeltaTime;
            Debug.Log("difference of speeds: " + (throwVelocity - m_rigidbody.velocity).magnitude);

            Debug.Log("Almost");
            if (bezierParameter >= 1f)
            {
                Debug.Log("Gotcha!");
                //bezierParameter = 0f;
                hammerState = HammerState.grabbed;
                deltaPosition = Vector3.zero;
                //m_rigidbody.useGravity = true;
                m_rigidbody.velocity = Vector3.zero;
                GrabAfterReturning();
            }
        }
        else
        {
            Vector3 force = (finalPosition - transform.position).normalized * maximumForceIntensityOnReturning;
            if(m_rigidbody.velocity.magnitude >= maximumSpeedWhenReturningWhenNotUsingBezierCurve)
            {
                force = force - (Vector3.Dot(force, m_rigidbody.velocity) * m_rigidbody.velocity / m_rigidbody.velocity.sqrMagnitude);
            }
            
            m_rigidbody.AddForce(force, ForceMode.VelocityChange);

            float angleToRotate = Vector3.Angle(transform.up, m_rigidbody.velocity);
            Vector3 rotationAxis = Vector3.Cross(transform.up, m_rigidbody.velocity);
            Quaternion rotation = Quaternion.AngleAxis(angleToRotate, rotationAxis);
            Vector3 rotatedAxisZ = rotation * transform.forward;
            m_rigidbody.MoveRotation(Quaternion.Lerp(transform.rotation, Quaternion.LookRotation(rotatedAxisZ, m_rigidbody.velocity), (float)(interpolateRawValue * interpolateRotationFactor)));

            throwVelocity = m_rigidbody.velocity;

            if((transform.position - finalPosition).sqrMagnitude <= (distanceToConsiderHammerReturnedWhenNotUsingBezierCurve * distanceToConsiderHammerReturnedWhenNotUsingBezierCurve))
            {
                //bezierParameter = 0f;
                hammerState = HammerState.grabbed;
                deltaPosition = Vector3.zero;
                //m_rigidbody.useGravity = true;
                GrabAfterReturning();
            }
        }
        
        throwAcceleration = Vector3.zero;
    }

    private void OnCollisionEnter(Collision collision)
    {
        if(!collision.gameObject.CompareTag("Enemy") && hammerState != HammerState.grabbed)
        {
            m_rigidbody.useGravity = true;
            hammerState = HammerState.released;
            shouldUseBezierCurve = true;
        }

        shouldLiftFromFloor = true;
    }

    private void OnCollisionStay(Collision collision)
    {
        if (!collision.gameObject.CompareTag("Enemy") && hammerState != HammerState.grabbed)
        {
            m_rigidbody.useGravity = true;
            hammerState = HammerState.released;
            shouldUseBezierCurve = true;
        }

        shouldLiftFromFloor = true;
    }

    protected virtual void InteractableObjectUsed(object sender, InteractableObjectEventArgs e)
    {
        DoSomething();
    }

    protected virtual void DoSomething()
    {

    }

    void CheckIfCallingHammerBack()
    {
        if (hammerState != HammerState.grabbed)
        {
            if (leftControllerEvents.gripClicked)
            {
                controllerToReturnIsRightHand = false;
                hammerState = HammerState.returning;
                finalPosition = leftController.transform.position;

                if (!controllerCallHammerPreviousStateClicked)
                {
                    currentLiftedHeight = 0f;
                    initialPositionToLiftFrom = transform.position;
                    if(m_rigidbody.velocity.magnitude < minimumSpeedToBeConsideredStopped)
                    {
                        shouldLiftFromFloor = true;
                    }
                    else
                    {
                        shouldLiftFromFloor = false;
                    }
                }

                controllerCallHammerPreviousStateClicked = true;
            }
            else if (rightControllerEvents.gripClicked)
            {
                controllerToReturnIsRightHand = true;
                hammerState = HammerState.returning;
                finalPosition = rightController.transform.position;

                if (!controllerCallHammerPreviousStateClicked)
                {
                    currentLiftedHeight = 0f;
                    initialPositionToLiftFrom = transform.position;
                    if(m_rigidbody.velocity.magnitude < minimumSpeedToBeConsideredStopped)
                    {
                        shouldLiftFromFloor = true;
                    }
                    else
                    {
                        shouldLiftFromFloor = false;
                    }
                }

                controllerCallHammerPreviousStateClicked = true;
            }
            else
            {
                if (controllerCallHammerPreviousStateClicked)
                {
                    Debug.Log("Called from CheckIfCallingHammerBack()");
                    m_rigidbody.isKinematic = false;
                    if (hammerState != HammerState.released)
                    {
                        hammerState = HammerState.flying;
                        shouldUseBezierCurve = false;
                    }
                    else
                    {
                        m_rigidbody.useGravity = true;
                        m_rigidbody.velocity = Vector3.zero;
                        m_rigidbody.angularVelocity = Vector3.zero;
                    }
                    //controllerCallHammerPreviousStateClicked = false;
                }
                bezierParameter = 0f;
                if (controllerToReturnIsRightHand)
                {
                    originalPosition = transform.position;
                }
                else
                {
                    originalPosition = transform.position;
                }
                controllerCallHammerPreviousStateClicked = false;
            }
        }
    }

    void GrabAfterReturning()
    {
        if (controllerToReturnIsRightHand)
        {
            rightAutoGrab.enabled = true;
            leftAutoGrab.enabled = false;
        }
        else
        {
            leftAutoGrab.enabled = true;
            rightAutoGrab.enabled = false;
        }

        Debug.Log("Tried to catch");
    }

    Vector3 GetBezierQuadraticCurvePoint(float t, Vector3 originalPosition, Vector3 curvePointReference, Vector3 finalPosition)
    {
        float u = 1 - t;
        float tt = t * t;
        float uu = u * u;
        Vector3 p = (uu * originalPosition) + (2 * u * t * curvePointReference) + (tt * finalPosition);
        return p;
    }

    void SetCurvePoint()
    {
        Vector3 controllerPosition = Vector3.zero;
        Vector3 offset;
        if (controllerToReturnIsRightHand)
        {
            if(rightController != null)
            {
                controllerPosition = rightController.transform.position;
            }
            else
            {
                if(leftController != null)
                {
                    controllerPosition = leftController.transform.position;
                }
            }
            offset = rightHandCurvePointOffset;
        }
        else
        {
            if (leftController != null)
            {
                controllerPosition = leftController.transform.position;
            }
            else
            {
                if (rightController != null)
                {
                    controllerPosition = rightController.transform.position;
                }
            }
            offset = leftHandCurvePointOffset;
        }

        if (!automaticallyCalculateBezierReferencePointDistance)
        {
            Vector3 hmdToController = 2f * Vector3.up;
            if (hmdTransform != null)
            {
                hmdToController = controllerPosition - hmdTransform.position;
            }
            curvePoint = controllerPosition + Quaternion.LookRotation(hmdToController, Vector3.up) * offset;
        }
        else
        {
            Vector3 delta = originalPosition - controllerPosition;
            Vector3 deltaDirectionOnXZPlane = new Vector3(delta.x, 0f, delta.z);
            Vector3 orthogonal;
            if (controllerToReturnIsRightHand)
            {
                orthogonal = (new Vector3(deltaDirectionOnXZPlane.z, 0f, -deltaDirectionOnXZPlane.x)).normalized * 0.5f * delta.magnitude;
            }
            else
            {
                orthogonal = (new Vector3(-deltaDirectionOnXZPlane.z, 0f, deltaDirectionOnXZPlane.x)).normalized * 0.5f * delta.magnitude;
            }
            curvePoint = controllerPosition + 0.5f * delta + orthogonal;
        }
        
        curvePoint.y = Mathf.Clamp(curvePoint.y, curvePointMinimumHeight, float.MaxValue);

        curvePointGameObject.transform.position = curvePoint;
    }

}
