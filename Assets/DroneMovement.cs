using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace DroneFlightController
{

    /// <summary>
    /// Flight Control System. Requires connection to Sensors and PID Controllers
    /// </summary>

    public class DroneMovement : MonoBehaviour
    {

        //-------------------------------------------------------------------------------------------
        //Drone Parts
        //-------------------------------------------------------------------------------------------

        private GameObject droneBody;
        private GameObject[] wings;
        private GameObject[] motors;
        private DroneSensors sensors;
        private float[] motorMix;

        //-------------------------------------------------------------------------------------------
        //Controls
        //-------------------------------------------------------------------------------------------

        public float desiredHeight = 0.0f;
        public float droneAcceleration = 0.0f;
        public int sensorUpdateFrecuency = 1; // 1 = every frame
        private int framesSinceLastUpdate = 0;

        //-------------------------------------------------------------------------------------------
        //PID Controllers
        //-------------------------------------------------------------------------------------------

        private PIDController PID_Height;
        private PIDController PID_Roll;
        private PIDController PID_Pitch;
        private PIDController PID_Yaw;

        //-------------------------------------------------------------------------------------------
        //Getters and Setters
        //-------------------------------------------------------------------------------------------

        /// <summary>
        /// Returns the droneBody object
        /// </summary>
        /// <returns>Drone Body</returns>

        public GameObject GetDroneBody()
        {
            return this.droneBody;
        }

        //-------------------------------------------------------------------------------------------
        //INITIALIZE EVERYTHING
        //-------------------------------------------------------------------------------------------


        // Start is called before the first frame update
        void Start()
        {
            //Initialize drone components
            droneBody = GameObject.Find("DroneBody");
            wings = new GameObject[] { GameObject.Find("FrontRightWing"), GameObject.Find("FrontLeftWing"), GameObject.Find("RearRightWing"), GameObject.Find("RearLeftWing") };
            motors = new GameObject[] { GameObject.Find("FrontRightMotor"), GameObject.Find("FrontLeftMotor"), GameObject.Find("RearRightMotor"), GameObject.Find("RearLeftMotor") };

            //Initialize Sensors
            this.sensors = new DroneSensors(this);
            this.UpdateSensors();

            //Initialize PID Controllers
            this.PID_Height = new PIDController(this, 0, 0, 0);
            this.PID_Roll = new PIDController(this, 0, 0, 0);
            this.PID_Pitch = new PIDController(this, 0, 0, 0);
            this.PID_Yaw = new PIDController(this, 0, 0, 0);

        }

        //-------------------------------------------------------------------------------------------
        //METHODS
        //-------------------------------------------------------------------------------------------

        



        ///<summary>
        ///Updates Drone's Sensors
        ///</summary>

        private void UpdateSensors()
        {
            this.sensors.UpdateSensors();
        }


        void FlightController()
        {

        }

        //Called every Physics frame
        private void FixedUpdate()
        {
            if (this.framesSinceLastUpdate >= this.sensorUpdateFrecuency - 1)
            {
                this.sensors.UpdateSensors();
                framesSinceLastUpdate = 0;
            }
            else
            {
                framesSinceLastUpdate++;
            }

            this.FlightController();
        }

        // Update is called once per frame
        void Update()
        {
            
        }
    }
    
    /// <summary>
    /// Receives a Goal State and Current State and outputs a signal towards the goal
    /// </summary>

    public class PIDController
    {
        private DroneMovement drone;

        private float P_Constant;
        private float I_Constant;
        private float D_Constant;

        private float goalState;
        private float currentState;
        private float error;
        private float previousState;
        private float integrator;
        private float derivative;

        public PIDController(DroneMovement drone, float P, float I, float D)
        {
            this.drone = drone;
            this.P_Constant = P;
            this.I_Constant = I;
            this.D_Constant = D;
            this.previousState = 0;
            this.integrator = 0;
            this.derivative = 0;
        }

        //-------------------------------------------------------------------------------------------
        //PROPORTION CONSTANTS SETTESR
        //-------------------------------------------------------------------------------------------

        void set_P(float P)
        {
            this.P_Constant = P;
        }

        void set_I(float I)
        {
            this.I_Constant = I;
        }

        void set_D(float D)
        {
            this.D_Constant = D;
        }

        //-------------------------------------------------------------------------------------------
        //PID FUNCTIONS
        //-------------------------------------------------------------------------------------------

        void CalculateDerivative(float currentState, float deltaTime)
        {
            this.derivative = (currentState - this.previousState) / deltaTime;
        }

        void UpdatePreviousData(float state)
        {
            this.previousState = state;
        }

        void Input(float goalState, float currentState, float deltaTime)
        {
            this.integrator += currentState;
            this.goalState = goalState;
            this.currentState = currentState;
            this.error = goalState - currentState;
            this.CalculateDerivative(currentState, deltaTime);
            this.UpdatePreviousData(currentState);
        }

        float Output()
        {
            float output = this.P_Constant * error + this.I_Constant * this.integrator + this.D_Constant * this.derivative;
            return output;
        }

    }

    /// <summary>
    /// Stores and provides information about the Drone
    /// </summary>

    public class DroneSensors
    {

        //MEASURES

        private float height;
        private Vector3 previousVelocity;
        private Vector3 velocity;
        private Vector3 acceleration;
        private Vector3 gravity = Physics.gravity;

        //OBJECTS

        private DroneMovement drone;
        private GameObject droneBody;
        
        //CONSTRUCTOR

        public DroneSensors(DroneMovement drone)
        {
            this.drone = drone;
            this.droneBody = drone.GetDroneBody();
            this.previousVelocity = new Vector3(0, 0, 0);
        }

        //------------------------------------------------------------------------------
        //GETTERS

        public float GetHeight()
        {
            return this.height;
        }

        public Vector3 GetAcceleration()
        {
            return this.acceleration;
        }

        public Vector3 GetGravity()
        {
            return this.gravity;
        }

        public Vector3 GetVelocity()
        {
            return this.velocity;
        }
        

        //------------------------------------------------------------------------------
        //CALCULATE

        private void CalculateHeight()
        {
            RaycastHit hitInfo;
            Physics.Raycast(this.droneBody.transform.position, Vector3.down, out hitInfo, 100);

            this.height = hitInfo.distance;
        }

        void CalculateAcceleration(Vector3 initialVel, Vector3 finalVel, float deltaTime)
        {
            this.acceleration = (finalVel - initialVel) / deltaTime;
        }

        void CalculateVelocity()
        {
            this.previousVelocity = this.velocity;
            this.velocity = this.droneBody.GetComponent<Rigidbody>().velocity;
        }

        public void UpdateSensors()
        {
            this.CalculateHeight();
            this.CalculateVelocity();
            this.CalculateAcceleration(this.previousVelocity, this.velocity, Time.fixedDeltaTime);
        }
    }
}
