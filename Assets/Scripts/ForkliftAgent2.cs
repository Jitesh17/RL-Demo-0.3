using System.Collections;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

    [RequireComponent(typeof (NewCarController))]
public class ForkliftAgent2 : Agent
{
    private NewCarController carAgent;
    [Tooltip("AreaDiameter")]
    public float AreaDiameter = 100f;
    // // public const float lastYaw = 0f;
    // /// <summary>
    // /// The block to be pushed to the goal.
    // /// </summary>
    public GameObject block;
    public Transform goal;
    public Transform fork; 
    // public Transform mast;
    public float speedTranslate = 0.2f; //Platform travel speed
    public Vector3 maxY = new Vector3(0f, 2.85f, 0f); //The maximum height of the platform
    public Vector3 minY = new Vector3(0f, -0.02f, 0f); 
    private float m_horizontalInput;
    private float m_verticalInput;

    private float m_handbrakeInput;

    Rigidbody m_BlockRb;  //cached on initialization

    public override void Initialize()
    {
        // goalDetect = block.GetComponent<GoalDetect>();
        // goalDetect.agent = this;
        Quaternion initialRotation = transform.rotation;
        // Cache the agent rigidbody
        // carAgent.m_Rigidbody = GetComponent<Rigidbody>();
        // Cache the block rigidbody
        m_BlockRb = block.GetComponent<Rigidbody>();
        // get the car controller
        carAgent = GetComponent<NewCarController>();
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
        // transform.rotation = Quaternion.Euler(0f, 0f, 0f);
        // carAgent.m_Rigidbody.velocity = Vector3.zero;
        // carAgent.m_Rigidbody.angularVelocity = Vector3.zero;


        goal.localPosition = new Vector3(10 + Random.value * 8, 1, Random.value * 8 + 10);
        // SetResetParameters();
    }



    /// <summary>
    /// Called every step of the engine. Here the agent takes an action.
    /// </summary>
    /*
    // Continuous actions
    public override void OnActionReceived(float[] actionsOut)

    {
        // Move the car
        carAgent.Move(actionsOut[0], actionsOut[1], actionsOut[1], actionsOut[2]);
        // Move the Fork
        if (actionsOut[3]>0)
            fork.transform.localPosition = Vector3.MoveTowards(fork.transform.localPosition, maxY, speedTranslate * Time.deltaTime);
        else if (actionsOut[3] <0)
            fork.transform.localPosition = Vector3.MoveTowards(fork.transform.localPosition, minY, speedTranslate * Time.deltaTime);
    */   

    // Discrete actions    
    public override void OnActionReceived(ActionBuffers actionBuffers)

    { 
        var action_accel = actionBuffers.DiscreteActions[0];
        var action_steer = actionBuffers.DiscreteActions[1];
        var action_brake = actionBuffers.DiscreteActions[2];
        var action_fork = actionBuffers.DiscreteActions[3];
        // Move the car
        carAgent.Move(action_steer, action_accel, action_accel, action_brake);
        // Move the Fork
        if (action_fork>0)
            fork.transform.localPosition = Vector3.MoveTowards(fork.transform.localPosition, maxY, speedTranslate * Time.deltaTime);
        else if (action_fork <0)
            fork.transform.localPosition = Vector3.MoveTowards(fork.transform.localPosition, minY, speedTranslate * Time.deltaTime);
        
    }
    public override void CollectObservations(VectorSensor sensor)
    {
        // sensor.AddObservation(transform.localRotation.normalized);
        // // sensor.AddObservation(carAgent.m_Rigidbody.position);
        // sensor.AddObservation(carAgent.m_Rigidbody.velocity.normalized);
        // sensor.AddObservation(m_BlockRb.position.normalized);
        // sensor.AddObservation(goal.localPosition.normalized);
        // sensor.AddObservation(transform.forward);

        Vector3 toBlock = m_BlockRb.position - transform.localPosition;
        Vector3 fromBlockToGoal = goal.localPosition - m_BlockRb.position;

        // sensor.AddObservation(fromBlockToGoal.normalized);  // (3 observations)
        // sensor.AddObservation(fromBlockToGoal.magnitude/ AreaDiameter); //  (1 observation)
        // sensor.AddObservation(carAgent.m_Rigidbody.rotation.normalized);  // (4 observations)
        // sensor.AddObservation(carAgent.m_Rigidbody.velocity.normalized);  // (3 observations)
        // sensor.AddObservation(toBlock.normalized);  // (3 observations)
        // sensor.AddObservation(toBlock.magnitude/ AreaDiameter);  //  (1 observation)
        // sensor.AddObservation(Vector3.Dot(toBlock.normalized, carAgent.m_Rigidbody.velocity.normalized)); //  (1 observation)
        // print(Vector3.Dot(toBlock.normalized, carAgent.m_Rigidbody.velocity.normalized));
    }
    
    // Discrete actions   
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var discreteActionsOut = actionsOut.DiscreteActions;
        discreteActionsOut[0] = 0;
        discreteActionsOut[1] = 0;
        discreteActionsOut[2] = 0;
        discreteActionsOut[3] = 0;

        if (Input.GetKey(KeyCode.W))
        {
            discreteActionsOut[0] = 1;
        }
        else if (Input.GetKey(KeyCode.S))
        {
            discreteActionsOut[0] = -1;
        }
        
        
        if (Input.GetKey(KeyCode.D))
        {
            discreteActionsOut[1] = 1;
        }
        else if (Input.GetKey(KeyCode.A))
        {
            discreteActionsOut[1] = -1;
        }

        if (Input.GetKey(KeyCode.Space))
        {
            discreteActionsOut[2] = 1;
            Debug.Log("Space key was pressed to apply brakes.");
        }
        
        if (Input.GetKey(KeyCode.UpArrow))
        {
            discreteActionsOut[3] = 1;
        }
        else if (Input.GetKey(KeyCode.DownArrow))
        {
            discreteActionsOut[3] = -1;
        }

    }
    /*
    // Continuous actions   
    public override void Heuristic(float[] actionsOut)
    {
        actionsOut[0] = Input.GetAxis("Horizontal");
        actionsOut[1] = Input.GetAxis("Vertical");

        actionsOut[2] = Input.GetAxis("Jump");
        actionsOut[3] = Input.GetAxis("Mouse Y");
        Debug.Log($"actionsOut[0]: {actionsOut[0]}");
        print(actionsOut[0]);
        Debug.Log(actionsOut[0]);
    }
    */
    
    void Update()
    {
        // print($"Agent's position: {transform.position} {transform.rotation} {transform.rotation.eulerAngles} \t\t Block's postion: {m_BlockRb.position} \t\t Goal's postion: {goal.localPosition}");
    }

}
