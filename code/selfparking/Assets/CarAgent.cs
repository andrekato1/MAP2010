/// TO-DO:
/// Randomize positions for cars and parking spot
/// Pintar vaga de verde só quando estiver perto o suficiente e alinhado


using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
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

    // Car settings
    public float maxMotorTorque = 800f; //200f
    public float maxSteeringAngle = 40f; //30f
    public float maxBrakeTorque = 500f;
    // public float maximumSpeed = 8f;
    float motor;
    float steering;                         
    float brake;

    // Observations - vectors normalized
    private float velocityXNorm;
    private float velocityZNorm;
    private Vector3 headingAngleNorm;
    private Vector3 agentPosNorm;
    private Vector3 parkingPosNorm;
    private Vector3 goalDistAgentNorm;
    // ------------------

    // Agent
    float goalDist;
    Vector3 defaultInitPos = new Vector3(11.34f, 7.527747f, 0f);
    bool initRandomPos = false;
    float parkingLotDirection;
    Rigidbody rb;
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
    public float goalReward = 1000f;
    public float alignedReward = 5000f;
    public float loseReward = -5000f;
    float sumRewards;
    // ------------------

    // Aux
    float parkingTime;
    float elapsed;
    bool isParking = false;
    public int numCollisions;
    public int numEpisode = 0;
    public int numSuccess = 0;
    public int numSuccessAligned = 0;
    private Color green = new Color(0.29412f, 0.70980f, 0.26275f);
    private Color red = new Color(1f, 0f, 0f);
    StatsRecorder sr;
    // ------------------

    // UI
    public TextMeshProUGUI throttleText;
    public TextMeshProUGUI steeringText;
    public TextMeshProUGUI totalRewards;
    public TextMeshProUGUI distanceReward;
    public TextMeshProUGUI headingReward;
    // ------------------

    // Rewards Curve
    public AnimationCurve fuzzyDistance;
    // ------------------

    /// <summary>
    /// Initialize the agent
    /// </summary>
    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        gos = GameObject.FindGameObjectsWithTag("parked_car");
        Vector3[] initialPos = new Vector3[gos.Length];
        sr = Academy.Instance.StatsRecorder;
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
        parkingTime = 0f;
        elapsed = 0f;
        numCollisions = 0;
        isParking = false;
        for (int i = 0; i < gos.Length; i++)
        {
            gos[i].transform.localPosition = parkedCarsInitialPos[i];
            gos[i].transform.localEulerAngles = new Vector3(0f, -90f, 0f);
        }

        rb.angularVelocity = Vector3.zero;
        rb.velocity = Vector3.zero;
        InitParking();
        transform.localPosition = defaultInitPos;
        transform.eulerAngles = new Vector3(0f, 0f, 0f);
        transform.localRotation = Quaternion.identity;
        sumRewards = 0;
        numEpisode++;
    }
    /// <summary>
    /// Randomize parked car positions and decide a parking spot for the agent
    /// </summary>
    public void InitParking()
    {
        //ChangeFloorColor(gray);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float carRotation = transform.localEulerAngles.y <= 180f ? transform.localEulerAngles.y : transform.localEulerAngles.y - 360f;
        motor = maxMotorTorque * actions.ContinuousActions[0];
        steering = maxSteeringAngle * actions.ContinuousActions[1];
        HandleMotor(motor, brake);
        HandleSteering(steering);
        UpdateWheels();
        AgentIsParking();

        goalDist = Vector3.Distance(transform.localPosition, parkingSpot.localPosition);
        headingAngleNorm = (parkingSpot.position - transform.position).normalized;
        float fuzzyGoalDist = fuzzyDistance.Evaluate(goalDist);
        //float carDirection = goalDist > .4f ? Vector3.Dot(headingAngleNorm, transform.forward) : 1f;
        parkingLotDirection = goalDist > .4f ? Vector3.Dot(transform.forward, parkingSpot.up) : 1f;
        //float fuzzyRewards = parkingLotDirection * carDirection * fuzzyGoalDist;

        //AddReward(carDirection*.1f);
        //sumRewards += carDirection*.1f;
        AddReward(parkingLotDirection * .1f);
        sumRewards += parkingLotDirection * .1f;
        AddReward(fuzzyGoalDist * .1f);
        sumRewards += fuzzyGoalDist * .1f;
        AddReward(-2f);
        sumRewards += -2f;
        
        if (Mathf.Abs(steering) > maxSteeringAngle || motor == 0)
        {
            AddReward(-1f / MaxStep);
            sumRewards += -1f / MaxStep;
        } 

        if (StepCount == MaxStep)
        {
            AddReward(loseReward);
            sumRewards += loseReward;
        }
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


    /// <summary>
    /// Checks if the agent is within the parking spot and if it is stopped for more than 1s
    /// </summary>
    public void AgentIsParking()
    {
        float carRotation = transform.localEulerAngles.y <= 180f ? transform.localEulerAngles.y : transform.localEulerAngles.y - 360f;

        if (goalDist <= .65f && parkingLotDirection > 0.9f && isParking == false)
        {
            parkingTime = Time.time;
            isParking = true;
            //AddReward(.5f);
            //sumRewards += .5f;
            parkingSpot.GetComponent<MeshRenderer>().material.color = green;
        }

        if (goalDist <= .65f && parkingLotDirection >= 0.9f && isParking == true)
        {
            elapsed = Time.time - parkingTime;
            //AddReward(.5f);
            //sumRewards += .5f;
            if (elapsed > .5f)
            {
                if (parkingLotDirection >= 0.97) {
                    AddReward(alignedReward);
                    sumRewards += goalReward;
                    Debug.Log("Car successfully parked with align bonus");
                    numSuccess++;
                    numSuccessAligned++;
                    sr.Add("Aligned Parking", 1);
                    EndEpisode();
                }
                AddReward(goalReward);
                sumRewards += goalReward;
                Debug.Log("Car successfully parked");
                numSuccess++;
                sr.Add("Aligned Parking", 1);
                EndEpisode();
            }
        }

        if (goalDist > .65f)
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

        //goalDistAgent = transform.InverseTransformVector(parkingSpot.localPosition - transform.localPosition);
        //goalDistAgentNorm = goalDistAgent.normalized;

        // 4 values
        sensor.AddObservation(headingAngleNorm);
        sensor.AddObservation(Vector3.Dot(headingAngleNorm, transform.forward));

        velocityXNorm = rb.velocity.normalized.x;
        velocityZNorm = rb.velocity.normalized.z;

        // 2 values total
        sensor.AddObservation(velocityXNorm);
        sensor.AddObservation(velocityZNorm);

        float carRotation = transform.localEulerAngles.y <= 180f ? transform.localEulerAngles.y : transform.localEulerAngles.y - 360f;

        // 2 values
        sensor.AddObservation(carRotation);

        // 2 values
        sensor.AddObservation(Vector3.Distance(transform.localPosition, parkingSpot.localPosition));
        sensor.AddObservation(Vector3.Dot(transform.forward, parkingSpot.up));
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Vertical");
        continuousActionsOut[1] = Input.GetAxis("Horizontal");
        //continuousActionsOut[2] = Input.GetAxis("Jump");
    }

    private void OnCollisionEnter(Collision collision)
    {
        AddReward(-1f);
        sumRewards += -1f;
        sr.Add("Collisions", 1);
        numCollisions++;
        if (numCollisions > 10)
        {
            EndEpisode();
        }
    }

    private void OnCollisionStay(Collision collision)
    {
        AddReward(-1f);
        sumRewards += -1f;
    }

    private void Update()
    {
        string roundedThrottle = motor.ToString("F3");
        string roundedSteering = steering.ToString("F3");
        string totalRewardsString = sumRewards.ToString("F3");
        throttleText.text = $"Throttle: {roundedThrottle}";
        steeringText.text = $"Steering: {roundedSteering}";
        totalRewards.text = $"Sum of Rewards: {totalRewardsString}";
        float b = fuzzyDistance.Evaluate(goalDist);
        float c = goalDist > .4f ? Vector3.Dot(transform.forward, parkingSpot.up) : 1f;
        distanceReward.text = $"Distance: {(b*.1f).ToString("F3")}";
        headingReward.text = $"Heading: {(c *.1f).ToString("F3")}";
        //Debug.DrawRay(rb.position,  (parkingSpot.position - transform.position).normalized, Color.blue);
    }
}