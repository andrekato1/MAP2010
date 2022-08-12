/// TO-DO:
/// Randomize positions for cars and parking spot
/// Pintar vaga de verde só quando estiver perto o suficiente e alinhado


using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

/// <summary>
/// Car ML Agent
/// </summary>
public class CarAgent : Agent
{
    // Car settings (from car controller)
    public WheelCollider frontLeftWheelCollider;
    public WheelCollider frontRightWheelCollider;
    public WheelCollider rearLeftWheelCollider;
    public WheelCollider rearRightWheelCollider;
    public Transform frontLeftWheelTransform;
    public Transform frontRightWheelTransform;
    public Transform rearLeftWheelTransform;
    public Transform rearRightWheelTransform;
    private float horizontalInput;
    private float verticalInput;
    private float steerAngle;
    private bool isBreaking;

    // Car settings
    public float maxMotorTorque = 200f;
    public float maxSteeringAngle = 30f;
    public float maxBrakeTorque = 500f;
    // public float maximumSpeed = 8f;
    float motor;
    float steering;                         
    float brake;

    // Observations - vectors normalized
    private float velocityXNorm;
    private float velocityZNorm;
    private float headingAngleNorm;
    private Vector3 agentPosNorm;
    private Vector3 parkingPosNorm;
    private Vector3 goalDistAgentNorm;
    // ------------------

    // Agent
    Vector3 zDirection;
    Vector3 goalDistAgent;
    float headingRelative;
    float headingAngle;
    float goalDist;
    Vector3 defaultInitPos = new Vector3(11.5f, 7.527747f, -0f);//new Vector3(7.894719f, 7.527747f, 0.570056f);
    bool initRandomPos = false;
    Rigidbody rb;
    float lastDistance;
    Vector3 lastVelocity;
    Vector3 acceleration;
    // ------------------

    // Parking spot
    public Transform parkingSpot;
    bool parkingRandomPos = false;
    float headingParkingTarget;
    float parkingDistance;
    Vector3 defaultParkingInitPos = new Vector3(-6.49f, .01f, -2.4f);
    GameObject[] gos;
    Vector3[] parkedCarsInitialPos;
    // ------------------

    // Rewards
    public float goalReward = 1f;
    bool isAligned;
    public float alignedReward = 3f;
    public float loseReward = -1f;
    float sumRewards;
    // ------------------

    // Aux
    float parkingTime;
    float elapsed;
    bool isParking = false;
    bool parked;
    public int numCollisions;
    public int numEpisode = 0;
    public int numSuccessEasyMode = 0;
    public int numSuccess = 0;
    bool easyMode;
    private Material material;
    private Color green = new Color(0.29412f, 0.70980f, 0.26275f);
    private Color gray = new Color(0.25490f, 0.25490f, 0.25490f);
    private Color red = new Color(1f, 0f, 0f);
    // ------------------

    /// <summary>
    /// Initialize the agent
    /// </summary>
    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        gos = GameObject.FindGameObjectsWithTag("parked_car");
        Vector3[] initialPos = new Vector3[gos.Length];
        for (int i = 0; i < gos.Length; i++)
        {
            initialPos[i] = gos[i].transform.localPosition;
        }
        parkedCarsInitialPos = initialPos;
    }

    /// <summary>
    /// Reset the agent on episode begin
    /// </summary>
    public override void OnEpisodeBegin()
    {
        isAligned = false;
        parkingTime = 0f;
        elapsed = 0f;
        numCollisions = 0;
        isParking = false;
        parked = false;
        lastVelocity = Vector3.zero;
        acceleration = Vector3.zero;
        zDirection = new Vector3(0, 0, 1);
        lastDistance = goalDist = Vector3.Distance(defaultInitPos, parkingSpot.localPosition);
        for (int i = 0; i < gos.Length; i++)
        {
            gos[i].transform.localPosition = parkedCarsInitialPos[i];
            gos[i].transform.localRotation = Quaternion.identity;
        }

        rb.angularVelocity = Vector3.zero;
        rb.velocity = Vector3.zero;
        InitParking();
        transform.localPosition = defaultInitPos;
        transform.eulerAngles = new Vector3(0f, -90f, 0f);//Quaternion.identity;
        easyMode = UnityEngine.Random.value > .5f;

        if (easyMode)
        {
            transform.localPosition = new Vector3(7.894719f, 7.527747f, 0.570056f);
            transform.eulerAngles = new Vector3(0f, 0f, 0f);
        }
        numEpisode++;
    }
    /// <summary>
    /// Randomize parked car positions and decide a parking spot for the agent
    /// </summary>
    public void InitParking()
    {
        ChangeFloorColor(gray);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        motor = maxMotorTorque * actions.ContinuousActions[0];//Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        steering = maxSteeringAngle * actions.ContinuousActions[1];//Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        brake = Mathf.Clamp(actions.ContinuousActions[2], 0f, 1f);
        HandleMotor(motor, brake);
        HandleSteering(steering);
        UpdateWheels();
        goalDist = Vector3.Distance(transform.localPosition, parkingSpot.localPosition);
        DistanceReward(goalDist, lastDistance);
        AddReward(-0.01f);
        AgentIsParking();

        if (Mathf.Abs(steering) > maxSteeringAngle) AddReward(-1f / MaxStep);
        if (motor <= 0) AddReward(-1f / MaxStep);
        if (StepCount == MaxStep) AddReward(loseReward);

        float carRotation = transform.localEulerAngles.y <= 180f ? transform.localEulerAngles.y : transform.localEulerAngles.y - 360f;

        if (goalDist < .4f && parked)
        {
            // Agent is sufficiently aligned with the parking spot, more reward
            if (Mathf.Abs(carRotation) < 5)
            {
                AddReward(alignedReward);
                if (easyMode)
                {
                    numSuccessEasyMode++;
                }
                else
                {
                    Debug.Log("Parked with align bonus");
                    numSuccess++;
                }
            }
            else
            {
                AddReward(goalReward);

                if (easyMode)
                {
                    numSuccessEasyMode++;
                }
                else
                {
                    Debug.Log("Parked with no align bonus");
                    numSuccess++;
                }

            }
            ChangeFloorColor(green);
            EndEpisode();
        }
        lastDistance = goalDist;
    }
    private void HandleSteering(float steering)
    {
        frontLeftWheelCollider.steerAngle = steering;
        frontRightWheelCollider.steerAngle = steering;
    }

    private void HandleMotor(float motor, float brake)
    {
        frontLeftWheelCollider.ConfigureVehicleSubsteps(5f, 100, 15);
        frontRightWheelCollider.ConfigureVehicleSubsteps(5f, 100, 15);
        rearLeftWheelCollider.ConfigureVehicleSubsteps(5f, 100, 15);
        rearRightWheelCollider.ConfigureVehicleSubsteps(5f, 100, 15);

        frontLeftWheelCollider.motorTorque = motor;
        frontRightWheelCollider.motorTorque = motor;

        float brakeForce = brake <= .8f ? 0f : brake * maxBrakeTorque;
        frontLeftWheelCollider.brakeTorque = brakeForce;
        frontRightWheelCollider.brakeTorque = brakeForce;
        rearLeftWheelCollider.brakeTorque = brake;
        rearRightWheelCollider.brakeTorque = brake;
    }
    private void UpdateWheels()
    {
        UpdateWheelPos(frontLeftWheelCollider, frontLeftWheelTransform);
        UpdateWheelPos(frontRightWheelCollider, frontRightWheelTransform);
        UpdateWheelPos(rearLeftWheelCollider, rearLeftWheelTransform);
        UpdateWheelPos(rearRightWheelCollider, rearRightWheelTransform);
    }

    private void UpdateWheelPos(WheelCollider wheelCollider, Transform trans)
    {
        Vector3 pos;
        Quaternion rot;
        wheelCollider.GetWorldPose(out pos, out rot);
        trans.rotation = rot;
        trans.position = pos;
    }

    public void DistanceReward(float currentDistance, float lastDistance)
    {
        if (currentDistance < lastDistance) AddReward(.1f);
        if (currentDistance > lastDistance) AddReward(-.2f);
    }

    /// <summary>
    /// Checks if the agent is within the parking spot and if it is stopped for more than 1s
    /// </summary>
    public void AgentIsParking()
    {
        float carRotation = transform.localEulerAngles.y <= 180f ? transform.localEulerAngles.y : transform.localEulerAngles.y - 360f;

        if (goalDist < .4f && Mathf.Abs(carRotation) < 15 && isParking == false)
        {
            parkingTime = Time.time;
            isParking = true;
            parkingSpot.GetComponent<MeshRenderer>().material.color = green;
        }

        if (goalDist < .4f && Mathf.Abs(carRotation) < 15 && isParking == true)
        {
            elapsed = Time.time - parkingTime;
            if (elapsed > 1f) parked = true;
        }

        if (goalDist > .4f)
        {
            isParking = false;
            parkingTime = 0f;
            elapsed = 0f;
            parkingSpot.GetComponent<MeshRenderer>().material.color = red;
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        agentPosNorm = transform.localPosition.normalized;
        parkingPosNorm = parkingSpot.transform.localPosition.normalized;

        // (x, y, z) 6 values total
        sensor.AddObservation(agentPosNorm);
        sensor.AddObservation(parkingPosNorm);

        goalDistAgent = transform.InverseTransformVector(parkingSpot.localPosition - transform.localPosition);
        goalDistAgentNorm = goalDistAgent.normalized;

        // 3 values
        sensor.AddObservation(goalDistAgentNorm);

        velocityXNorm = rb.velocity.normalized.x;
        velocityZNorm = rb.velocity.normalized.z;

        // 2 values total
        sensor.AddObservation(velocityXNorm);
        sensor.AddObservation(velocityZNorm);

        acceleration = (rb.velocity - lastVelocity) / Time.fixedDeltaTime;
        lastVelocity = rb.velocity;
        headingAngle = Vector3.SignedAngle(zDirection, transform.forward, Vector3.up);
        headingAngleNorm = Vector3.SignedAngle(zDirection, transform.forward, Vector3.up) / 100f;
        float carRotation = transform.localEulerAngles.y <= 180f ? transform.localEulerAngles.y : transform.localEulerAngles.y - 360f;

        // 1 value
        //sensor.AddObservation(headingAngleNorm);
        sensor.AddObservation(carRotation);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Vertical");
        continuousActionsOut[1] = Input.GetAxis("Horizontal");
        continuousActionsOut[2] = Input.GetAxis("Jump");
    }

    private void OnCollisionEnter(Collision collision)
    {
        AddReward(-1f);
        numCollisions++;
        if (numCollisions > 10)
        {
            Debug.Log("Episode ended due excessive crashing");
            EndEpisode();
        }
    }

    private void OnCollisionStay(Collision collision)
    {
        AddReward(-.5f);
    }

    private void Update()
    {
        Debug.DrawLine(rb.position, parkingSpot.position, Color.green);
    }

    private void ChangeFloorColor(Color color)
    {
        MeshRenderer mr = GameObject.FindGameObjectWithTag("floor").GetComponent<MeshRenderer>();
        material = mr.material;
        material.SetColor("_Color", color);
    }
}
