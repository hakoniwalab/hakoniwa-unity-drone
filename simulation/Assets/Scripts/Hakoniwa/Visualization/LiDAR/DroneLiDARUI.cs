using UnityEngine;
using System;
using System.Collections.Generic;

namespace hakoniwa.visualization.lidar
{
    /// <summary>
    /// Renders a runtime UI using IMGUI for controlling the Drone3DLiDARVisualizer.
    /// </summary>
    public class DroneLiDARUI : MonoBehaviour
    {
        private Drone3DLiDARVisualizer visualizer;

        // UI state variables
        private Rect windowRect = new Rect(0, 0, 300, 450); // Initial size, position will be set in Start()
        private string currentColorName = "Green";

        private readonly Dictionary<string, Color> colorPresets = new Dictionary<string, Color>
        {
            { "Green", Color.green },
            { "Red", Color.red },
            { "Blue", Color.blue },
            { "Orange", new Color(1.0f, 0.5f, 0.0f) },
            { "Magenta", Color.magenta }
        };

        void Start()
        {
            // Find the visualizer component in the scene
            visualizer = FindFirstObjectByType<Drone3DLiDARVisualizer>();
            if (visualizer == null)
            {
                Debug.LogWarning("DroneLiDARUI: Drone3DLiDARVisualizer not found in scene. UI will be disabled.");
            }

            // If visualizer is found, initialize current color name based on its default color
            if (visualizer != null)
            {
                UpdateCurrentColorName(visualizer.GetColorForCurrentRayMode());
            }

            // Set the initial position of the window to the bottom-left
            windowRect.x = 20;
            windowRect.y = Screen.height - windowRect.height - 20;
        }

        void OnGUI()
        {
            if (visualizer == null)
            {
                // If visualizer is not found, don't draw the UI
                return;
            }

            // Register the window to be drawn by the GUI system
            windowRect = GUILayout.Window(0, windowRect, DrawLidarControlWindow, "LiDAR Control");
        }

        /// <summary>
        /// This function is called by the GUI system to draw the contents of the window.
        /// </summary>
        void DrawLidarControlWindow(int windowID)
        {
            // Allow the window to be dragged
            GUI.DragWindow(new Rect(0, 0, 10000, 20));

            // Use GUILayout for automatic arrangement of controls
            GUILayout.BeginVertical();

            // --- Toggles ---
            visualizer.showRays = GUILayout.Toggle(visualizer.showRays, "Show Rays");
            visualizer.showDebugInfo = GUILayout.Toggle(visualizer.showDebugInfo, "Show Debug Info");

            GUILayout.Space(10);

            // --- Ray Mode ---
            GUILayout.Label($"Ray Mode: {visualizer.rayVisualization}");
            if (GUILayout.Button("Cycle Ray Mode"))
            {
                CycleRayMode();
            }

            GUILayout.Space(10);

            // --- Ray Color ---
            GUILayout.Label($"Ray Color: {currentColorName}");
            GUILayout.BeginHorizontal();
            foreach (var colorEntry in colorPresets)
            {
                if (GUILayout.Button(colorEntry.Key))
                {
                    SetRayColor(colorEntry.Value, colorEntry.Key);
                }
            }
            GUILayout.EndHorizontal();

            GUILayout.Space(10);

            // --- Sliders ---
            // Max Rendered Rays
            GUILayout.Label($"Max Rendered Rays: {visualizer.maxRenderedRays}");
            visualizer.maxRenderedRays = (int)GUILayout.HorizontalSlider(visualizer.maxRenderedRays, 10, 20000);

            // Ray Sampling
            GUILayout.Label($"Ray Sampling: {visualizer.raySampling}");
            visualizer.raySampling = (int)GUILayout.HorizontalSlider(visualizer.raySampling, 1, 20);

            // Particle Size
            GUILayout.Label($"Particle Size: {visualizer.particleSize:F2}");
            visualizer.SetParticleSize(GUILayout.HorizontalSlider(visualizer.particleSize, 0.01f, 0.5f));

            GUILayout.Space(10);
            
            // --- Debug Info ---
            if (visualizer.showDebugInfo)
            {
                GUILayout.Label(visualizer.GetStats());
            }


            GUILayout.EndVertical();
        }

        private void CycleRayMode()
        {
            // Get all values of the enum
            var values = (Drone3DLiDARVisualizer.RayVisualizationMethod[])Enum.GetValues(typeof(Drone3DLiDARVisualizer.RayVisualizationMethod));
            // Find the index of the current mode
            int currentIndex = Array.IndexOf(values, visualizer.rayVisualization);
            // Increment index and wrap around if it goes past the end
            int nextIndex = (currentIndex + 1) % values.Length;
            // Set the new mode
            visualizer.rayVisualization = values[nextIndex];
            visualizer.RefreshRayVisualization();

            // Update color display as it might be different for the new mode
            UpdateCurrentColorName(visualizer.GetColorForCurrentRayMode());
        }

        private void SetRayColor(Color color, string colorName)
        {
            visualizer.SetColorForCurrentRayMode(color);
            visualizer.RefreshRayVisualization();
            currentColorName = colorName;
        }

        private void UpdateCurrentColorName(Color color)
        {
            foreach (var entry in colorPresets)
            {
                if (entry.Value == color)
                {
                    currentColorName = entry.Key;
                    return;
                }
            }
            // If the color is not a preset, show its RGB values
            currentColorName = $"RGB({(int)(color.r * 255)}, {(int)(color.g * 255)}, {(int)(color.b * 255)})";
        }
    }
}
