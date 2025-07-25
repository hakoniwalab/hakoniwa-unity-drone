using System.Collections.Generic;
using System.IO;
using hakoniwa.objects.core.sensors;
using Newtonsoft.Json;
using UnityEngine;

namespace hakoniwa.drone
{
    public class DroneConfig : MonoBehaviour
    {
        private string audioPath;
        [System.Serializable]
        public class DroneConfigData
        {
            public Dictionary<string, DroneDetails> drones;
        }

        [System.Serializable]
        public class DroneDetails
        {
            public string audio_rotor_path;
            public Dictionary<string, DroneLidarDetails> LiDARs;
        }
        [System.Serializable]
        public class DroneLidarDetails
        {
            public bool Enabled;
            public int NumberOfChannels;
            public int RotationsPerSecond;
            public int PointsPerSecond;
            public float VerticalFOVUpper;
            public float VerticalFOVLower;
            public float HorizontalFOVStart;
            public float HorizontalFOVEnd;
            public bool DrawDebugPoints;
            public float MaxDistance;
            public float X;
            public float Y;
            public float Z;
            public float Roll;
            public float Pitch;
            public float Yaw;
        }
        private DroneConfigData loadedData = null;
        public string drone_config_path = "./drone_config.json";
        public void LoadDroneConfig(string droneName)
        {
            string filePath = drone_config_path;
            Debug.Log("Looking for config file at: " + filePath);

            if (File.Exists(filePath))
            {
                string dataAsJson = File.ReadAllText(filePath);
                loadedData = JsonConvert.DeserializeObject<DroneConfigData>(dataAsJson);

                if (loadedData != null && loadedData.drones != null)
                {
                    if (loadedData.drones.ContainsKey(droneName))
                    {
                        audioPath = loadedData.drones[droneName].audio_rotor_path;
                        Debug.Log("Audio Path for " + droneName + ": " + audioPath);
                    }
                    else
                    {
                        Debug.LogError("Drone configuration for " + droneName + " not found.");
                    }
                }
                else
                {
                    Debug.LogError("Drone configurations are missing or corrupt. Check JSON structure.");
                }
            }
            else
            {
                Debug.LogError("Cannot find drone_config.json file at: " + filePath);
            }
        }
        private bool GetParam(string droneName, string name, out LiDAR3DParams param)
        {
            param = new LiDAR3DParams();
            if (loadedData == null)
            {
                return false;
            }
            if (loadedData.drones.ContainsKey(droneName))
            {
                if (loadedData.drones[droneName].LiDARs.ContainsKey(name))
                {
                    Debug.Log("found param: " + name);
                    param.Enabled = loadedData.drones[droneName].LiDARs[name].Enabled;
                    param.NumberOfChannels = loadedData.drones[droneName].LiDARs[name].NumberOfChannels;
                    param.RotationsPerSecond = loadedData.drones[droneName].LiDARs[name].RotationsPerSecond;
                    param.PointsPerSecond = loadedData.drones[droneName].LiDARs[name].PointsPerSecond;
                    param.MaxDistance = loadedData.drones[droneName].LiDARs[name].MaxDistance;
                    param.VerticalFOVUpper = loadedData.drones[droneName].LiDARs[name].VerticalFOVUpper;
                    param.VerticalFOVLower = loadedData.drones[droneName].LiDARs[name].VerticalFOVLower;
                    param.HorizontalFOVStart = loadedData.drones[droneName].LiDARs[name].HorizontalFOVStart;
                    param.HorizontalFOVEnd = loadedData.drones[droneName].LiDARs[name].HorizontalFOVEnd;
                    param.DrawDebugPoints = loadedData.drones[droneName].LiDARs[name].DrawDebugPoints;
                    return true;
                }
                return false;
            }
            else
            {
                return false;
            }
        }
        public void SetLidarPosition(string droneName)
        {
            var lidars = this.GetComponentsInChildren<ILiDAR3DController>();
            foreach (var ilidar in lidars)
            {
                var lidar = ((MonoBehaviour)ilidar).gameObject;
                Debug.Log("Found Lidar: " + lidar.transform.parent.gameObject.name);
                LiDAR3DParams param;
                if (this.GetParam(droneName, lidar.transform.parent.gameObject.name, out param))
                {
                    ilidar.SetParams(param);
                    //pos
                    float x = loadedData.drones[droneName].LiDARs[lidar.transform.parent.gameObject.name].X;
                    float y = loadedData.drones[droneName].LiDARs[lidar.transform.parent.gameObject.name].Y;
                    float z = loadedData.drones[droneName].LiDARs[lidar.transform.parent.gameObject.name].Z;
                    float y_off = lidar.transform.parent.parent.position.y;
                    Vector3 v = new Vector3(x, y, z);
                    Vector3 v_unity = ConvertRos2Unity(v);
                    lidar.transform.parent.localPosition = v_unity;
                    Debug.Log("v: " + v_unity);
                    //angle
                    float roll = loadedData.drones[droneName].LiDARs[lidar.transform.parent.gameObject.name].Roll;
                    float pitch = loadedData.drones[droneName].LiDARs[lidar.transform.parent.gameObject.name].Pitch;
                    float yaw = loadedData.drones[droneName].LiDARs[lidar.transform.parent.gameObject.name].Yaw;
                    Vector3 euler_angle = new Vector3(roll, pitch, yaw);
                    Vector3 euler_angle_unity = -ConvertRos2Unity(euler_angle);
                    Debug.Log("euler_angle: " + euler_angle_unity);
                    lidar.transform.parent.localEulerAngles = euler_angle_unity;
                }
            }
        }
        private Vector3 ConvertRos2Unity(Vector3 ros_data)
        {
            return new Vector3(
                -ros_data.y, // unity.x
                ros_data.z, // unity.y
                ros_data.x  // unity.z
                );
        }

    }
}
