using System.Collections;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class ForkliftAgent : Agent
{

    [Tooltip("Force to apply when moving")]
    public float moveForce = 2f;

    [Tooltip("Speed to rotate around the up axis")]
    public float yawSpeed = 100f;
   
    [Tooltip("AreaDiameter")]
    public float AreaDiameter = 100f;
    // public const float lastYaw = 0f;
    /// <summary>
    /// The block to be pushed to the goal.
    /// </summary>
    public GameObject block;

    /// <summary>
    /// The goal to push the block to.
    /// </summary>
    public Transform goal;

    Rigidbody m_BlockRb;  //cached on initialization
    Rigidbody m_AgentRb;  //cached on initialization

    public override void Initialize()
    {
        // goalDetect = block.GetComponent<GoalDetect>();
        // goalDetect.agent = this;
        Quaternion initialRotation = transform.rotation;
        // Cache the agent rigidbody
        m_AgentRb = GetComponent<Rigidbody>();
        // Cache the block rigidbody
        m_BlockRb = block.GetComponent<Rigidbody>();
        // Get the ground's bounds
        // areaBounds = ground.GetComponent<Collider>().bounds;
        // // Get the ground renderer so we can change the material when a goal is scored
        // m_GroundRenderer = ground.GetComponent<Renderer>();
        // // Starting material
        // m_GroundMaterial = m_GroundRenderer.material;

        // m_ResetParams = Academy.Instance.EnvironmentParameters;

        // SetResetParameters();
    }

    /// <summary>
    /// Resets the block position and velocities.
    /// </summary>
    void ResetBlock()
    {
        // Get a random position for the block.
        // block.transform.position = GetRandomSpawnPos();

        // Reset block velocity back to zero.
        if (UnityEngine.Random.value > .5f)
        {
            m_BlockRb.position = new Vector3(-10 - Random.value * 8, 1, 10 - Random.value * 8);
        }
        m_BlockRb.velocity = Vector3.zero;

        // Reset block angularVelocity back to zero.
        m_BlockRb.angularVelocity = Vector3.zero;
    }

    /// <summary>
    /// In the editor, if "Reset On Done" is checked then AgentReset() will be
    /// called automatically anytime we mark done = true in an agent script.
    /// </summary>
    public override void OnEpisodeBegin()
    {
        var rotation = Random.Range(0, 4);
        var rotationAngle = rotation * 90f;
        // area.transform.Rotate(new Vector3(0f, rotationAngle, 0f));

        ResetBlock();
        // transform.position = GetRandomSpawnPos();
        if (UnityEngine.Random.value > .5f)
        {
            transform.position = Vector3.zero;
        }
        // transform.position = Vector3.zero;
        transform.rotation = Quaternion.Euler(0f, 0f, 0f);;
        m_AgentRb.velocity = Vector3.zero;
        m_AgentRb.angularVelocity = Vector3.zero;


        goal.localPosition = new Vector3(10 + Random.value * 8, 1, Random.value * 8 + 10);
        // SetResetParameters();
    }


    /// <summary>
    /// Moves the agent according to the selected action.
    /// </summary>
    public void MoveAgent(ActionSegment<int> act)
    {
        var dirToGo = Vector3.zero;
        var rotateDir = Vector3.zero;

        var action = act[0];

        switch (action)
        {
            case 1:
                dirToGo = transform.forward * 1f;
                break;
            case 2:
                dirToGo = transform.forward * -1f;
                break;
            case 3:
                rotateDir = transform.up * 1f;
                break;
            case 4:
                rotateDir = transform.up * -1f;
                break;
            case 5:
                dirToGo = transform.right * -0.75f;
                break;
            case 6:
                dirToGo = transform.right * 0.75f;
                break;
        }
        transform.Rotate(rotateDir, Time.fixedDeltaTime * yawSpeed);
        m_AgentRb.AddForce(dirToGo * moveForce,
            ForceMode.VelocityChange);
    }


    /// <summary>
    /// Called every step of the engine. Here the agent takes an action.
    /// </summary>
    public override void OnActionReceived(ActionBuffers actionBuffers)

    {
        print($"Agent's position: {transform.position} {transform.rotation} {transform.rotation.eulerAngles} \t\t Block's postion: {m_BlockRb.position} \t\t Goal's postion: {goal.localPosition}");
        // Move the agent using the action.
        MoveAgent(actionBuffers.DiscreteActions);


        Vector3 toBlock = m_BlockRb.position - transform.localPosition;
        AddReward(Vector3.Dot(toBlock.normalized, m_AgentRb.velocity.normalized));
        // Vector3 fromBlockToGoal = goal.localPosition - m_BlockRb.position;
        // AddReward(Vector3.Dot(fromBlockToGoal.normalized, m_BlockRb.velocity.normalized));
        // float distanceToBlock = Vector3.Distance(m_BlockRb.position, m_AgentRb.position);
        // if (distanceToBlock > 1.42f)
        // {
        //     AddReward(-0.01f);
        // }
        // Goal
        float distanceBlockToGoal = Vector3.Distance(m_BlockRb.position, goal.localPosition);
        // if (distanceBlockToGoal < 2f)
        if (toBlock.magnitude < 4f)
        {
            AddReward(100f);
            // SetReward(1.0f);
            EndEpisode();
        }
        else
        {
            AddReward(-0.0001f);
        }

        // Get the current rotation
        Vector3 rotationVector = transform.rotation.eulerAngles;
        // AddReward(-0.01f * Mathf.Abs(rotationVector.x)/90);
        // AddReward(-0.01f * Mathf.Abs(rotationVector.z)/90);

        float angleError = 20f;
        bool angleXOk = (Mathf.Abs(rotationVector.x) > angleError) && (Mathf.Abs(rotationVector.x) < 360 - angleError);
        bool angleZOk = (Mathf.Abs(rotationVector.z) > angleError) && (Mathf.Abs(rotationVector.z) < 360 - angleError);
        if (this.transform.localPosition.y < -5 || angleXOk || angleZOk)
                {
                    AddReward(-1f);
                    EndEpisode();
                }
        // Penalty given each step to encourage agent to finish task quickly.
        // AddReward(-1f / MaxStep);
    }
    public override void CollectObservations(VectorSensor sensor)
    {
        // sensor.AddObservation(transform.localRotation.normalized);
        // // sensor.AddObservation(m_AgentRb.position);
        // sensor.AddObservation(m_AgentRb.velocity.normalized);
        // sensor.AddObservation(m_BlockRb.position.normalized);
        // sensor.AddObservation(goal.localPosition.normalized);
        // sensor.AddObservation(transform.forward);

        Vector3 toBlock = m_BlockRb.position - transform.localPosition;
        Vector3 fromBlockToGoal = goal.localPosition - m_BlockRb.position;

        // sensor.AddObservation(fromBlockToGoal.normalized);  // (3 observations)
        // sensor.AddObservation(fromBlockToGoal.magnitude/ AreaDiameter); //  (1 observation)
        sensor.AddObservation(m_AgentRb.rotation.normalized);  // (4 observations)
        sensor.AddObservation(m_AgentRb.velocity.normalized);  // (3 observations)
        sensor.AddObservation(toBlock.normalized);  // (3 observations)
        sensor.AddObservation(toBlock.magnitude/ AreaDiameter);  //  (1 observation)
        sensor.AddObservation(Vector3.Dot(toBlock.normalized, m_AgentRb.velocity.normalized)); //  (1 observation)
        // print(Vector3.Dot(toBlock.normalized, m_AgentRb.velocity.normalized));
    }
    
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var discreteActionsOut = actionsOut.DiscreteActions;
        discreteActionsOut[0] = 0;
        if (Input.GetKey(KeyCode.D))
        {
            discreteActionsOut[0] = 3;
        }
        else if (Input.GetKey(KeyCode.W))
        {
            discreteActionsOut[0] = 1;
        }
        else if (Input.GetKey(KeyCode.A))
        {
            discreteActionsOut[0] = 4;
        }
        else if (Input.GetKey(KeyCode.S))
        {
            discreteActionsOut[0] = 2;
        }
    }


    void Update()
    {
        // print($"Agent's position: {transform.position} {transform.rotation} {transform.rotation.eulerAngles} \t\t Block's postion: {m_BlockRb.position} \t\t Goal's postion: {goal.localPosition}");
    }

}
