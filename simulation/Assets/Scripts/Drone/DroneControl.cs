using hakoniwa.drone.sim;
using hakoniwa.objects.core;
using hakoniwa.objects.core.sensors;
using hakoniwa.pdu.interfaces;
using UnityEngine;

namespace hakoniwa.drone
{
    public enum DroneControlInputType
    {
        PS4,
        Xbox,
        Xr
    }
    public interface IDroneControlOp
    {
        public void DoInitialize(string robotName);
        public int PutRadioControlButton(int index, int value);
        public int PutFlightModeChangeButton(int index, int value);
        public int PutHorizontal(int index, double value);
        public int PutForward(int index, double value);
        public int PutHeading(int index, double value);
        public int PutVertical(int index, double value);
        public bool GetMagnetRequest(out bool magnet_on);
        public void PutMagnetStatus(bool magnet_on, bool contact_on);
        public void DoFlush();
    }

    public class DroneControl : MonoBehaviour
    {
        public string robotName = "Drone";
        public DroneControlInputType input_type;
        public double stick_strength = 0.1;
        public double stick_yaw_strength = 1.0;
        private IDroneInput controller_input;
        public bool magnet_on = false;
        public GameObject grabberObject;
        private IBaggageGrabber grabber;
        public bool forceGrab = false;
        private IDroneControlOp droneControlOp = null;
        public bool isPlayer = true;

        public IDroneInput GetDroneInput()
        {
            return controller_input;
        }

        public bool IsMagnetOn()
        {
            return magnet_on;
        }
        private void Awake()
        {
            if (isPlayer)
            {
                droneControlOp = new DroneControlRc();
            }
            else
            {
                droneControlOp = this.GetComponentInChildren<IDroneControlOp>();
            }
            droneControlOp.DoInitialize(robotName);
        }

        private void Start()
        {
            if (grabberObject != null)
            {
                grabber = grabberObject.GetComponentInChildren<IBaggageGrabber>();
                Debug.Log("grabber: " + grabber);
            }
            else
            {
                Debug.Log("Gabber is not found.");
            }

            // DroneControlInputType selection & builder
            if (input_type == DroneControlInputType.Xr)
            {
                // Quest Input Controller
                controller_input = HakoDroneXrInputManager.Instance;
            }
            else
            {
                // Gamepad Input Controller
                controller_input = HakoDroneInputManager.Instance;
            }        
        }

        public float move_step = 1.0f;  // ���̓����̃X�e�b�v��
        private float camera_move_button_time_duration = 0f;
        public float camera_move_button_threshold_speedup = 1.0f;
        public bool is_pressed_up = false;
        public bool is_pressed_down = false;

        public void HandleCameraControl(ICameraController camera_controller, IPduManager pduManager)
        {
            {
                /*
                 * Camera Image Rc request
                 */
                if (controller_input.IsYButtonPressed() && pduManager != null)
                {
                    Debug.Log("SHOT!!");
                    camera_controller.Scan();
                    camera_controller.WriteCameraDataPdu(pduManager);
                }
                if (controller_input.IsUpButtonPressed())
                {
                    is_pressed_up = true;
                }
                else if (controller_input.IsUpButtonReleased())
                {
                    is_pressed_up = false;
                }
                if (is_pressed_up)
                {
                    camera_move_button_time_duration += Time.fixedDeltaTime;
                    if (camera_move_button_time_duration > camera_move_button_threshold_speedup)
                    {
                        camera_controller.RotateCamera(-move_step * 3f);
                    }
                    else
                    {
                        camera_controller.RotateCamera(-move_step);
                    }

                }
                if (controller_input.IsDownButtonPressed())
                {
                    is_pressed_down = true;
                }
                else if (controller_input.IsDownButtonReleased())
                {
                    is_pressed_down = false;
                }


                if (is_pressed_down)
                {
                    camera_move_button_time_duration += Time.fixedDeltaTime;
                    if (camera_move_button_time_duration > camera_move_button_threshold_speedup)
                    {
                        camera_controller.RotateCamera(move_step * 3f);
                    }
                    else
                    {
                        camera_controller.RotateCamera(move_step);
                    }
                }

                if (!is_pressed_down && !is_pressed_up)
                {
                    camera_move_button_time_duration = 0f;
                }
            }
        }
        public void HandleInput()
        {
            Vector2 leftStick = controller_input.GetLeftStickInput();
            Vector2 rightStick = controller_input.GetRightStickInput();
            float horizontal = rightStick.x;
            float forward = rightStick.y;
            float yaw = leftStick.x;
            float pitch = leftStick.y;

            bool mag_on = false;
            bool mag_req = droneControlOp.GetMagnetRequest(out mag_on);
            if (controller_input.IsAButtonPressed())
            {
                droneControlOp.PutRadioControlButton(0, 1);
            }
            else if (controller_input.IsAButtonReleased())
            {
                droneControlOp.PutRadioControlButton(0, 0);
            }
            if (controller_input.IsXButtonPressed())
            {
                droneControlOp.PutFlightModeChangeButton(0, 1);
            }
            else if (controller_input.IsXButtonReleased())
            {
                droneControlOp.PutFlightModeChangeButton(0, 0);
            }
            if (controller_input.IsBButtonReleased())
            {
                Debug.Log("Bbutton released");
                magnet_on = IsMagnetOn() ? false : true;
                if (grabber != null)
                {
                    if (magnet_on)
                    {
                        grabber.Grab(forceGrab);
                    }
                    else
                    {
                        grabber.Release();
                    }
                }

            }
            else if (mag_req)
            {
                if (mag_on)
                {
                    Debug.Log("Request: mag_on");
                    grabber.Grab(forceGrab);
                }
                else
                {
                    Debug.Log("Request: mag_off");
                    grabber.Release();
                }
                magnet_on = mag_on;
            }
            if (grabber  != null)
            {
                droneControlOp.PutMagnetStatus(magnet_on, grabber.IsGrabbed());
            }
            droneControlOp.PutHorizontal(0, horizontal * stick_strength);
            droneControlOp.PutForward(0, -forward * stick_strength);
            droneControlOp.PutHeading(0, yaw * stick_yaw_strength);
            droneControlOp.PutVertical(0, -pitch * stick_strength);

            droneControlOp.DoFlush();
        }

    }
}
