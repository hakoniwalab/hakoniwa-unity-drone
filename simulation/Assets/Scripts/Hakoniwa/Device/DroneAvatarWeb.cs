using hakoniwa.drone;
using hakoniwa.drone.sim;
using hakoniwa.objects.core;
using hakoniwa.objects.core.sensors;
using hakoniwa.pdu.core;
using hakoniwa.pdu.interfaces;
using hakoniwa.pdu.msgs.geometry_msgs;
using hakoniwa.pdu.msgs.hako_mavlink_msgs;
using hakoniwa.pdu.msgs.hako_msgs;
using hakoniwa.sim;
using NUnit.Framework;
using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using UnityEngine;

public class DroneAvatarWeb : MonoBehaviour, IHakoniwaWebObject, IDroneBatteryStatus
{
    public string robotName = "Drone";
    public string pdu_name_propeller = "motor";
    public string pdu_name_pos = "pos";
    public string pdu_name_collision = "impulse";
    public string pdu_name_disturbance = "disturb";
    public string pdu_name_status = "status";
    private DroneCollision drone_collision;
    public GameObject body;
    private DronePropeller drone_propeller;
    private DroneControl drone_control;
    public bool useBattery = true;
    public string pdu_name_battery = "battery";
    private hakoniwa.pdu.msgs.hako_msgs.HakoBatteryStatus battery_status;
    public DroneCameraController camera_controller;
    private DroneConfig droneConfig;
    private ILiDAR3DController[] lidars = null;
    private Wind wind;
    public double sea_level_atm = 1.0;
    public double sea_level_temperature = 15.0;
    public DroneLedController[] leds;
    public FlightModeLedController[] flight_mode_leds;
    public PropellerWindController[] propeller_winds;

    void Start()
    {
        if (body == null)
        {
            throw new Exception("Body is not assigned");
        }
        drone_propeller = this.GetComponentInChildren<DronePropeller>();
        if (drone_propeller == null)
        {
            throw new Exception("Can not found drone propeller");
        }
        Debug.Log("max rotation : " + drone_propeller.maxRotationSpeed);
        drone_control = this.GetComponentInChildren<DroneControl>();
        if (drone_control == null)
        {
            Debug.Log("not found DroneControl");
        }
        /*
         * Drone Config
         */
        droneConfig = this.GetComponentInChildren<DroneConfig>();
        if (droneConfig)
        {
            droneConfig.LoadDroneConfig(robotName);
        }
        /*
         * Collision
         */
        drone_collision = this.GetComponentInChildren<DroneCollision>();
        if (drone_collision != null)
        {
            Debug.Log("collision is attached.");
        }
    }
    private float[] prev_controls = new float[4];
    private IPduManager pduManager;
    void FixedUpdate()
    {
        if (pduManager == null)
        {
            pduManager = WebServerBridge.Instance.Get();
            if (pduManager == null)
            {
                Debug.LogWarning("Can not find pduManager...");
                return;
            }
        }

        /*
         * Position
         */
        IPdu pdu_pos = pduManager.ReadPdu(robotName, pdu_name_pos);
        if (pdu_pos == null)
        {
            //Debug.Log("Can not get pdu of pos");
        }
        else
        {
            Twist pos = new Twist(pdu_pos);
            //Debug.Log($"Twist ({pos.linear.x} {pos.linear.y} {pos.linear.z})");
            UpdatePosition(pos);

            if (lidars == null)
            {
                var local_lidars = this.GetComponentsInChildren<ILiDAR3DController>();
                if (local_lidars != null)
                {
                    if (droneConfig)
                    {
                        droneConfig.SetLidarPosition(robotName);
                    }
                    foreach (var lidar in local_lidars)
                    {
                        lidar.DoInitialize(robotName, pduManager);
                    }
                    lidars = local_lidars;
                }
            }
        }

        /*
         * Propeller
         */
        float propellerRotation = 0;
        IPdu pdu_propeller = pduManager.ReadPdu(robotName, pdu_name_propeller);
        if (pdu_propeller == null)
        {
            //Debug.Log("Can not get pdu of propeller");
            drone_propeller.Rotate(prev_controls[0], prev_controls[1], prev_controls[2], prev_controls[3]);
        }
        else
        {
            HakoHilActuatorControls propeller = new HakoHilActuatorControls(pdu_propeller);
            for (int i = 0; i < 4; i++)
            {
                prev_controls[i] = propeller.controls[i];
            }
            //Debug.Log($"c1: {propeller.controls[0]} c2: {propeller.controls[1]} c3: {propeller.controls[2]} c4: {propeller.controls[3]}");
            drone_propeller.Rotate((float)propeller.controls[0], (float)propeller.controls[1], (float)propeller.controls[2], (float)propeller.controls[3]);
            propellerRotation = (float)propeller.controls[0];
        }
        /*
         * Battery
         */
        if (useBattery)
        {
            IPdu pdu_battery = pduManager.ReadPdu(robotName, pdu_name_battery);
            if (pdu_battery != null)
            {
                battery_status = new hakoniwa.pdu.msgs.hako_msgs.HakoBatteryStatus(pdu_battery);
            }
        }
        /*
         * Camera
         */
        if (camera_controller != null)
        {
            camera_controller.DoControl(pduManager);
        }
        /*
         * LiDAR
         */
        if (lidars != null)
        {
            foreach (var lidar in lidars)
            {
                lidar.DoControl(pduManager);
            }
        }
        /*
         * Collision
         */
        if (drone_collision)
        {
            var col = drone_collision.GetImpulseCollision();
            if (col.collision)
            {
                INamedPdu pdu_col = pduManager.CreateNamedPdu(robotName, pdu_name_collision);
                hakoniwa.pdu.msgs.hako_msgs.ImpulseCollision impulseCollision = new hakoniwa.pdu.msgs.hako_msgs.ImpulseCollision(pdu_col);
                impulseCollision.collision = true;
                impulseCollision.is_target_static = col.isTargetStatic;
                impulseCollision.restitution_coefficient = col.restitutionCoefficient;
                impulseCollision.self_contact_vector.x = col.selfContactVector.x;
                impulseCollision.self_contact_vector.y = col.selfContactVector.y;
                impulseCollision.self_contact_vector.z = col.selfContactVector.z;
                impulseCollision.normal.x = col.normal.x;
                impulseCollision.normal.y = col.normal.y;
                impulseCollision.normal.z = col.normal.z;
                impulseCollision.target_contact_vector.x = col.targetContactVector.x;
                impulseCollision.target_contact_vector.y = col.targetContactVector.y;
                impulseCollision.target_contact_vector.z = col.targetContactVector.z;
                impulseCollision.target_velocity.x = col.targetVelocity.x;
                impulseCollision.target_velocity.y = col.targetVelocity.y;
                impulseCollision.target_velocity.z = col.targetVelocity.z;
                impulseCollision.target_angular_velocity.x = col.targetAngularVelocity.x;
                impulseCollision.target_angular_velocity.y = col.targetAngularVelocity.y;
                impulseCollision.target_angular_velocity.z = col.targetAngularVelocity.z;
                impulseCollision.target_euler.x = col.targetEuler.x;
                impulseCollision.target_euler.y = col.targetEuler.y;
                impulseCollision.target_euler.z = col.targetEuler.z;
                impulseCollision.target_inertia.x = col.targetInertia.x;
                impulseCollision.target_inertia.y = col.targetInertia.y;
                impulseCollision.target_inertia.z = col.targetInertia.z;
                impulseCollision.target_mass = col.targetMass;
                pduManager.WriteNamedPdu(pdu_col);
                pduManager.FlushNamedPdu(pdu_col);
            }
        }
        /*
         * Disturbance
         */
        if (wind != null)
        {
            IPdu pdu_disturb = pduManager.ReadPdu(robotName, pdu_name_disturbance);
            if (pdu_disturb != null)
            {
                Disturbance disturb = new Disturbance(pdu_disturb);
                UnityEngine.Vector3 wind_dir = new UnityEngine.Vector3(
                    -(float)disturb.d_wind.value.y,
                    (float)disturb.d_wind.value.z,
                    (float)disturb.d_wind.value.x
                    );
                wind.wind_direction = wind_dir;
                sea_level_temperature = disturb.d_temp.value;
                sea_level_atm = disturb.d_atm.sea_level_atm;
                //Debug.Log("sea_leve_atm = " + sea_level_atm);
            }

        }
        /*
         * Drone Status
         */
        IPdu pdu_status = pduManager.ReadPdu(robotName, pdu_name_status);
        if (pdu_status != null)
        {
            DroneStatus drone_status = new DroneStatus(pdu_status);
            //Debug.Log("internal_state: " + drone_status.internal_state);
            /*
             * Leds
             */
            if (leds.Length > 0)
            {
                if (propellerRotation > 0)
                {
                    foreach (var led in leds)
                    {
                        switch (drone_status.internal_state)
                        {
                            case 0:
                                led.SetMode(DroneLedController.DroneMode.TAKEOFF);
                                break;
                            case 1:
                                led.SetMode(DroneLedController.DroneMode.HOVER);
                                break;
                            case 2:
                                led.SetMode(DroneLedController.DroneMode.LANDING);
                                break;
                            default:
                                break;
                        }
                    }
                }
                else
                {
                    foreach (var led in leds)
                    {
                        led.SetMode(DroneLedController.DroneMode.DISARM);
                    }
                }
            }
            if (flight_mode_leds.Length > 0)
            {
                foreach (var led in flight_mode_leds)
                {
                    if (drone_status.flight_mode == 0)
                    {
                        led.SetMode(FlightModeLedController.FlightMode.ATTI);
                    }
                    else
                    {
                        led.SetMode(FlightModeLedController.FlightMode.GPS);
                    }
                }
            }
            /*
             * Propeller Winds
             */
            if (propeller_winds.Length > 0)
            {
                UnityEngine.Vector3 w = new UnityEngine.Vector3(
                    (float)drone_status.propeller_wind.x,
                    (float)drone_status.propeller_wind.y,
                    (float)drone_status.propeller_wind.z
                );
                foreach (var wind in propeller_winds)
                {
                    wind.SetWindVelocityFromRos(w);
                }
            }

        }

    }
    void Update()
    {
        /*
         * DroneControl
         */
        if (drone_control != null)
        {
            //Debug.Log("Do Drone Control..");
            drone_control.HandleInput();
            if ((pduManager != null) && (camera_controller != null))
            {
                drone_control.HandleCameraControl(camera_controller.GetCameraController(), pduManager);
            }
        }

    }

    private void UpdatePosition(Twist pos)
    {
        UnityEngine.Vector3 unity_pos = new UnityEngine.Vector3();
        unity_pos.z = (float)pos.linear.x;
        unity_pos.x = -(float)pos.linear.y;
        unity_pos.y = (float)pos.linear.z;
        body.transform.position = unity_pos;
        //Debug.Log("pos: " + body.transform.position);

        float rollDegrees = Mathf.Rad2Deg * (float)pos.angular.x * 1;
        float pitchDegrees = Mathf.Rad2Deg * (float)pos.angular.y * 1;
        float yawDegrees = Mathf.Rad2Deg * (float)pos.angular.z;

        UnityEngine.Quaternion rotation = UnityEngine.Quaternion.Euler(pitchDegrees, -yawDegrees, -rollDegrees);
        body.transform.rotation = rotation;
    }

    public async Task DeclarePduAsync()
    {
        var pdu_manager = WebServerBridge.Instance.Get();
        if (pdu_manager == null)
        {
            throw new Exception("Can not get Pdu Manager");
        }
        //this.robotName = robot_name;
        var ret = await pdu_manager.DeclarePduForRead(robotName, pdu_name_pos);
        Debug.Log("declare pdu pos: " + ret);
        ret = await pdu_manager.DeclarePduForRead(robotName, pdu_name_propeller);
        Debug.Log("declare pdu propeller: " + ret);
        /*
         * Collision
         */
        if (drone_collision)
        {
            ret = await pdu_manager.DeclarePduForWrite(robotName, pdu_name_collision);
            if (ret == false)
            {
                throw new ArgumentException($"Can not declare pdu for write: {robotName} {pdu_name_collision}");
            }
        }

        /*
         * Battery
         */
        if (useBattery)
        {
            ret = await pdu_manager.DeclarePduForRead(robotName, pdu_name_battery);
            if (ret == false)
            {
                throw new ArgumentException($"Can not declare pdu for read: {robotName} {pdu_name_battery}");
            }
        }

        foreach (UnityEngine.Transform child in this.transform)
        {
            var subObjects = child.GetComponentsInChildren<IHakoniwaWebObject>();
            foreach (var obj in subObjects)
            {
                if ((UnityEngine.Object)obj != (UnityEngine.Object)this
                    && obj is MonoBehaviour mb
                    && mb.enabled
                    && mb.gameObject.activeInHierarchy)
                {
                    await obj.DeclarePduAsync();
                }

            }
        }
        /*
         * Camera
         */
        if (camera_controller)
        {
            camera_controller.GetCameraController().DelclarePdu(robotName, pdu_manager);
        }
        /*
         * LiDAR
         */
        // lidars's initialization must be done on drone's position is received...
        lidars = null;
        var local_lidars = this.GetComponentInChildren<ILiDAR3DController>();
        if (local_lidars != null)
        {
            ret = await pduManager.DeclarePduForWrite(robotName, Default3DLiDARController.pdu_name_lidar_pos);
            if (ret == false)
            {
                throw new ArgumentException($"Can not declare pdu for read: {robotName} {Default3DLiDARController.pdu_name_lidar_pos}");
            }
            ret = await pduManager.DeclarePduForWrite(robotName, Default3DLiDARController.pdu_name_lidar_point_cloud);
            if (ret == false)
            {
                throw new ArgumentException($"Can not declare pdu for read: {robotName} {Default3DLiDARController.pdu_name_lidar_point_cloud}");
            }
        }
        /*
         * Disturbance
         */
        wind = this.GetComponentInChildren<Wind>();
        if (wind != null)
        {
            ret = await pdu_manager.DeclarePduForRead(robotName, pdu_name_disturbance);
            if (ret == false)
            {
                throw new ArgumentException($"Can not declare pdu for read: {robotName} {pdu_name_disturbance}");
            }
            Debug.Log("Disturbance pdu for read is success: " + robotName + "/" +  pdu_name_disturbance);
        }
        /*
         * Drone Status
         */
        ret = await pdu_manager.DeclarePduForRead(robotName, pdu_name_status);
        if (ret == false)
        {
            throw new ArgumentException($"Can not declare pdu for read: {robotName} {pdu_name_status}");
        }
        /*
         * Leds
         */
        if (leds.Length > 0)
        {
            foreach (var led in leds)
            {
                led.SetMode(DroneLedController.DroneMode.DISARM);
            }
        }
        if (flight_mode_leds.Length > 0)
        {
            foreach (var led in flight_mode_leds)
            {
                led.SetMode(FlightModeLedController.FlightMode.GPS);
            }

        }
        /*
         * Propeller Winds
         */
        if (propeller_winds.Length > 0)
        {
            foreach (var wind in propeller_winds)
            {
                wind.SetWindVelocityFromRos(UnityEngine.Vector3.zero);
            }

        }
    }

    public double get_full_voltage()
    {
        if (battery_status != null)
        {
            return battery_status.full_voltage;
        }
        return 0;
    }

    public double get_curr_voltage()
    {
        if (battery_status != null)
        {
            return battery_status.curr_voltage;
        }
        return 0;
    }

    public uint get_status()
    {
        if (battery_status != null)
        {
            return battery_status.status;
        }
        return 0;
    }

    public uint get_cycles()
    {
        if (battery_status != null)
        {
            return battery_status.cycles;
        }
        return 0;

    }

    public double get_temperature()
    {
        if (battery_status != null)
        {
            return battery_status.curr_temp;
        }
        return 0;
    }

    // 外部から設定される高度（m）
    public double Altitude = 121.321;

    public double get_atmospheric_pressure()
    {
        return AtmosphericPressure.PascalToAtm(
            AtmosphericPressure.ConvertAltToBaro(
                Altitude + this.transform.position.y,
                sea_level_atm,
                sea_level_temperature));
    }
}
