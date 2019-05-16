 using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LightningTrail : MonoBehaviour
{
    ParticleSystem particleSystem;
    ParticleSystem.MainModule mainModule;
    bool canEmit = false;
    [SerializeField] float minimumTimeBetweenEmissions = 0.1f;
    double timeSinceLastEmission = 0f;
    Vector3 lastHammerPosition = Vector3.zero;
    [SerializeField] GameObject hammer;

    double hammerSpeed = 0f;
    Vector3 hammerMotionVector = Vector3.zero;
    [SerializeField] float minimumHammerSpeedForEmission = 2f;
    double timeSinceLastSpeedMesurement = 0f;
    [SerializeField] float timeBetweenSpeedMesurements = 0.1f;

    [Space]
    float maximumTrailLength = 3f;

    private void Awake()
    {
        particleSystem = GetComponent<ParticleSystem>();
        mainModule = particleSystem.main;   
    }


    // Update is called once per frame
    void Update()
    {
        timeSinceLastSpeedMesurement += Time.deltaTime;
        if (timeSinceLastSpeedMesurement >= timeBetweenSpeedMesurements)
        {
            timeSinceLastSpeedMesurement = 0f;
            lastHammerPosition = hammer.transform.position;
        }

        /*if(timeSinceLastSpeedMesurement != 0f)
        {
            hammerMotionVector = hammer.transform.position - lastHammerPosition;
            hammerSpeed = hammerMotionVector.magnitude / timeSinceLastSpeedMesurement;
            Debug.Log(hammerSpeed);
        }*/

        timeSinceLastEmission += Time.deltaTime;

        if(timeSinceLastEmission >= minimumTimeBetweenEmissions)
        {
            hammerMotionVector = hammer.transform.position - lastHammerPosition;
            if(timeSinceLastSpeedMesurement != 0f)
            {
                hammerSpeed = hammerMotionVector.magnitude / timeSinceLastSpeedMesurement;
                //Debug.Log(hammerSpeed);                
                if (hammerSpeed >= minimumHammerSpeedForEmission)
                {
                    timeSinceLastEmission = 0f;
                    timeSinceLastSpeedMesurement = 0f;
                    lastHammerPosition = hammer.transform.position;
                    Emit();
                }
            }
        }
    }

    void Emit()
    {
        //Debug.Log("Emission!");
        particleSystem.transform.rotation = Quaternion.LookRotation(-hammerMotionVector);
        mainModule.startSizeY = Mathf.Clamp(hammerMotionVector.magnitude, 0f, maximumTrailLength);
        mainModule.startSizeX = mainModule.startSizeY;
        int numberOfParticles = Random.Range(2, 4);
        particleSystem.Emit(numberOfParticles);
    }
}
