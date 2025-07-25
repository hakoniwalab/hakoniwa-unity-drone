using System;
using hakoniwa.objects.core;
using hakoniwa.pdu.interfaces;
using hakoniwa.pdu.msgs.geometry_msgs;
using hakoniwa.pdu.msgs.hako_mavlink_msgs;
using hakoniwa.pdu.msgs.hako_msgs;
using hakoniwa.sim;
using hakoniwa.sim.core;
using UnityEngine;

namespace hakoniwa.drone.sim
{
    public class DroneAvatar : MonoBehaviour, IHakoObject, IDroneBatteryStatus, IMovableObject
    {
        IHakoPdu hakoPdu;
        public string robotName = "Drone";
        public string pdu_name_propeller = "motor";
        public string pdu_name_pos = "pos";
        public string pdu_name_touch_sensor = "baggage_sensor";
        public string pdu_name_collision = "impulse";
        public string pdu_name_battery = "battery";
        public string pdu_name_disturbance = "disturb";
        public string pdu_name_status = "status";
        public bool useBattery = true;
        public GameObject body;
        public Rigidbody rd;
        public bool useTouchSensor;
        private TouchSensor touchSensor;
        private DroneCollision drone_collision;
        private hakoniwa.pdu.msgs.hako_msgs.HakoBatteryStatus battery_status;
        private CameraController cameraController;
        private BaggageGrabber baggageGrabber;
        private GameController gameController;
        private DroneConfig droneConfig;
        private LiDAR3DController[] lidars;
        private Wind wind;
        public double sea_level_atm = 1.0;
        public double sea_level_temperature = 15.0;
        public DroneLedController[] leds;
        public FlightModeLedController[] flight_mode_leds;
        public PropellerWindController[] propeller_winds;

        private DronePropeller drone_propeller;

        public void EventInitialize()
        {
            Debug.Log("Event Initialize");
            if (body == null)
            {
                throw new Exception("Body is not assigned");
            }
            if (rd == null)
            {
                throw new Exception("Can not find rigidbody on " + this.gameObject.name);
            }
            drone_propeller = this.GetComponentInChildren<DronePropeller>();
            if (drone_propeller == null)
            {
                Debug.Log("Can not found drone propeller");
            }
            drone_collision = this.GetComponentInChildren<DroneCollision>();
            if (drone_collision != null)
            {
                Debug.Log("collision is attached.");
            }

            hakoPdu = HakoAsset.GetHakoPdu();
            /*
             * Position
             */
            var ret = hakoPdu.DeclarePduForRead(robotName, pdu_name_pos);
            if (ret == false)
            {
                throw new ArgumentException($"Can not declare pdu for read: {robotName} {pdu_name_pos}");
            }
            /*
             * Propeller
             */
            if (drone_propeller)
            {
                ret = hakoPdu.DeclarePduForRead(robotName, pdu_name_propeller);
                if (ret == false)
                {
                    throw new ArgumentException($"Can not declare pdu for read: {robotName} {pdu_name_propeller}");
                }
            }
            /*
             * Battery
             */
            if (useBattery)
            {
                ret = hakoPdu.DeclarePduForRead(robotName, pdu_name_battery);
                if (ret == false)
                {
                    throw new ArgumentException($"Can not declare pdu for read: {robotName} {pdu_name_battery}");
                }
            }
            /*
             * TouchSensor
             */
            if (useTouchSensor)
            {
                ret = hakoPdu.DeclarePduForWrite(robotName, pdu_name_touch_sensor);
                if (ret == false)
                {
                    throw new ArgumentException($"Can not declare pdu for write: {robotName} {pdu_name_touch_sensor}");
                }
                touchSensor = this.GetComponentInChildren<TouchSensor>();
                if (touchSensor == null)
                {
                    throw new ArgumentException($"Can not find touch sensor: {robotName} {pdu_name_touch_sensor}");
                }
            }
            /*
             * Collision
             */
            if (drone_collision)
            {
                ret = hakoPdu.DeclarePduForWrite(robotName, pdu_name_collision);
                if (ret == false)
                {
                    throw new ArgumentException($"Can not declare pdu for write: {robotName} {pdu_name_collision}");
                }
            }
            /*
             * GameController
             */
            gameController = this.GetComponentInChildren<GameController>();
            if (gameController)
            {
                gameController.DoInitialize(robotName, hakoPdu);
            }
            /*
             * Camera
             */
            cameraController = this.GetComponentInChildren<CameraController>();
            if (cameraController)
            {
                cameraController.DoInitialize(robotName, hakoPdu);
            }
            /*
             * Baggage
             */
            baggageGrabber = this.GetComponentInChildren<BaggageGrabber>();
            if (baggageGrabber)
            {
                baggageGrabber.DoInitialize(robotName, hakoPdu);
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
             * LiDAR
             */
            lidars = this.GetComponentsInChildren<LiDAR3DController>();
            if (lidars != null)
            {
                if (droneConfig)
                {
                    droneConfig.SetLidarPosition(robotName);
                }
                foreach(var lidar in lidars)
                {
                    lidar.DoInitialize(robotName, hakoPdu);
                }
            }
            /*
             * Disturbance
             */
            wind = this.GetComponentInChildren<Wind>();
            if (wind != null)
            {
                ret = hakoPdu.DeclarePduForRead(robotName, pdu_name_disturbance);
                if (ret == false)
                {
                    throw new ArgumentException($"Can not declare pdu for read: {robotName} {pdu_name_disturbance}");
                }
            }
            /*
             * Drone Status
             */
            ret = hakoPdu.DeclarePduForRead(robotName, pdu_name_status);
            if (ret == false)
            {
                throw new ArgumentException($"Can not declare pdu for read: {robotName} {pdu_name_status}");
            }
            /*
             * Leds
             */
            if (leds.Length > 0)
            {
                foreach (var led in leds) {
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

        public void EventReset()
        {
            //nothing to do
        }

        public void EventStart()
        {
            //nothing to do
        }

        public void EventStop()
        {
            //nothing to do
        }

        public void EventTick()
        {
            var pduManager = hakoPdu.GetPduManager();
            if (pduManager == null)
            {
                return;
            }

            /*
             * Position
             */
            IPdu pdu_pos = pduManager.ReadPdu(robotName, pdu_name_pos);
            if (pdu_pos == null)
            {
                Debug.Log("Can not get pdu of pos");
            }
            else
            {
                Twist pos = new Twist(pdu_pos);
                //Debug.Log($"Twist ({pos.linear.x} {pos.linear.y} {pos.linear.z})");
                UpdatePosition(pos);
            }
            float propellerRotation = 0;
            if (drone_propeller)
            {
                /*
                 * Propeller
                 */
                IPdu pdu_propeller = pduManager.ReadPdu(robotName, pdu_name_propeller);
                if (pdu_propeller == null)
                {
                    //Debug.Log("Can not get pdu of propeller");
                }
                else
                {
                    HakoHilActuatorControls propeller = new HakoHilActuatorControls(pdu_propeller);
                    //Debug.Log("c1: " + propeller.controls[0]);
                    drone_propeller.Rotate((float)propeller.controls[0], (float)propeller.controls[1], (float)propeller.controls[2], (float)propeller.controls[3]);
                    propellerRotation = (float)propeller.controls[0];
                }
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

            if (touchSensor)
            {
                INamedPdu pdu_touch_sensor = pduManager.CreateNamedPdu(robotName, pdu_name_touch_sensor);
                hakoniwa.pdu.msgs.std_msgs.Bool is_touched = new hakoniwa.pdu.msgs.std_msgs.Bool(pdu_touch_sensor);
                is_touched.data = touchSensor.IsTouched();
                pduManager.WriteNamedPdu(pdu_touch_sensor);
                pduManager.FlushNamedPdu(pdu_touch_sensor);
            }
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
             * GameController
             */
            if (gameController)
            {
                gameController.DoControl(pduManager);
            }
            /*
             * Camera
             */
            if (cameraController)
            {
                cameraController.DoControl(pduManager);
            }
            /*
             * Baggage
             */
            if (baggageGrabber)
            {
                baggageGrabber.DoControl(pduManager);
            }
            /*
             * LiDAR
             */
            if (lidars != null)
            {
                foreach(var lidar in lidars)
                {
                    lidar.DoControl(pduManager);
                }
            }
            /*
             * Disturbance
             */ 
            if (wind != null)
            {
                IPdu pdu_disturb = pduManager.ReadPdu(robotName, pdu_name_disturbance);
                if (pdu_disturb == null)
                {
                    Debug.Log("Can not get pdu of pdu_disturb");
                }
                else
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
                    Debug.Log("Wind: " + w);
                    foreach (var wind in propeller_winds)
                    {
                        wind.SetWindVelocityFromRos(w);
                    }
                }

            }
        }
        public bool enableLerp = false;
        private void UpdatePosition(Twist pos)
        {
            UnityEngine.Vector3 unity_pos = new UnityEngine.Vector3();
            unity_pos.z = (float)pos.linear.x;
            unity_pos.x = -(float)pos.linear.y;
            unity_pos.y = (float)pos.linear.z;
            //body.transform.position = unity_pos;
            //Debug.Log("pos: " + body.transform.position);

            float rollDegrees = Mathf.Rad2Deg * (float)pos.angular.x;
            float pitchDegrees = Mathf.Rad2Deg * (float)pos.angular.y;
            float yawDegrees = Mathf.Rad2Deg * (float)pos.angular.z;

            UnityEngine.Quaternion targetRotation = UnityEngine.Quaternion.Euler(pitchDegrees, -yawDegrees, -rollDegrees);
            //body.transform.rotation = rotation;

            if (enableLerp)
            {
                float speed = 8.0f;
                float step = speed * Time.deltaTime;

                // 位置をLerp
                UnityEngine.Vector3 startPosition = this.rd.position;
                UnityEngine.Vector3 endPosition = unity_pos;
                this.rd.MovePosition(UnityEngine.Vector3.Lerp(startPosition, endPosition, step));

                // 回転をLerp
                UnityEngine.Quaternion startRotation = this.rd.rotation;
                this.rd.MoveRotation(UnityEngine.Quaternion.Lerp(startRotation, targetRotation, step));
            }
            else
            {
                this.rd.MovePosition(unity_pos);
                this.rd.MoveRotation(targetRotation);
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

        UnityEngine.Vector3 IMovableObject.GetPosition()
        {
            return this.body.transform.position;
        }

        UnityEngine.Vector3 IMovableObject.GetEulerDeg()
        {
            return this.body.transform.eulerAngles;
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
}
