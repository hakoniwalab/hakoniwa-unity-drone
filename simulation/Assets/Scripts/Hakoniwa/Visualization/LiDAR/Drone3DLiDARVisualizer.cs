using System;
using UnityEngine;
using hakoniwa.pdu.interfaces;
using hakoniwa.pdu.msgs.sensor_msgs;

namespace hakoniwa.visualization.lidar
{
    /// <summary>
    /// ドローン用3D LiDAR可視化スクリプト
    /// 
    /// 使い方:
    /// 1. このスクリプトをLiDARオブジェクトにアタッチ
    /// 2. Visualization Modeを選択:
    ///    - ParticleSystem: パーティクルシステムで点群を表示（推奨）
    ///    - Gizmos: Gizmosで点を表示（エディタのみ、デバッグ用）
    ///    - DebugRays: Debug.DrawRayで線を表示（エディタのみ）
    /// 3. Robot Nameに対象ロボット名を設定
    /// 4. 必要に応じてパラメータを調整
    /// 
    /// 注意:
    /// - ParticleSystemモードはGameビューとSceneビューの両方で動作
    /// - Gizmos/DebugRaysモードはSceneビューのみ（エディタ限定）
    /// </summary>
    public class Drone3DLiDARVisualizer : MonoBehaviour
    {
        [Header("可視化設定")]
        [Tooltip("可視化モード")]
        public VisualizationMode visualizationMode = VisualizationMode.ParticleSystem;
        
        [Tooltip("対象ロボット名")]
        public string robotName = "Drone";
        
        [Header("ParticleSystem設定")]
        [Tooltip("パーティクルサイズ")]
        [Range(0.01f, 0.5f)]
        public float particleSize = 0.05f;
        
        [Tooltip("距離による色分け")]
        public bool colorByDistance = true;
        
        [Tooltip("近距離色（距離0m）")]
        public Color nearColor = Color.blue;
        
        [Tooltip("遠距離色（最大距離）")]
        public Color farColor = Color.red;
        
        [Tooltip("最大表示距離")]
        [Range(1f, 50f)]
        public float maxDisplayDistance = 10f;
        
        [Header("フィルタリング設定")]
        [Tooltip("最小距離フィルタ")]
        [Range(0f, 5f)]
        public float minDistance = 0.1f;
        
        [Tooltip("最大距離フィルタ")]
        [Range(1f, 50f)]
        public float maxDistance = 20f;
        
        [Tooltip("Z軸フィルタを有効化")]
        public bool enableZFilter = true;
        
        [Tooltip("Z軸最小値")]
        [Range(-10f, 10f)]
        public float zMin = -1f;
        
        [Tooltip("Z軸最大値")]
        [Range(-10f, 10f)]
        public float zMax = 3f;
        
        [Header("デバッグ設定")]
        [Tooltip("情報をコンソールに出力")]
        public bool showDebugInfo = false;
        
        [Tooltip("更新レート（0=毎フレーム）")]
        [Range(0, 60)]
        public int targetUpdateRate = 10;
        
        [Tooltip("パーティクルマテリアル")]
        public Material particleMaterial;

        [Header("Ray Visualization")]
        [Tooltip("LiDARのレイ（照射）をGameビューで可視化する")]
        public bool showRays = false;
        public RayVisualizationMethod rayVisualization = RayVisualizationMethod.GLLines; // デフォルトはGL.LINES
        [Tooltip("描画する最大レイ数（サンプリング前）")]
        public int maxRenderedRays = 2000;
        [Tooltip("サンプリング間隔（例: 4 = 4点に1本描画）")]
        public int raySampling = 4;
        [Tooltip("LineRendererの線幅")]
        public float lineWidth = 0.02f;
        [Tooltip("LineRendererやGL.LINES用のマテリアル（未設定時はフォールバック）")]
        public Material rayMaterial;
        [Tooltip("Particle方式用のパーティクルシステムのプレハブ（未設定時はスクリプトが生成）")]
        public ParticleSystem rayParticleSystemPrefab;

        // Per-mode colors for ray visualization (configurable at runtime)
        [Tooltip("Color used for LineRenderer rays")]
        public Color lineRendererColor = Color.green;
        [Tooltip("Color used for GL.LINES rays")]
        public Color glLinesColor = Color.green;
        [Tooltip("Color used for ParticleBeams")]
        public Color particleBeamsColor = Color.green;

        public enum RayVisualizationMethod { None, LineRenderer, GLLines, ParticleBeams }

        // 内部変数
        private new ParticleSystem particleSystem;
        private ParticleSystem.Particle[] particles;
        private Vector3[] pointCloud;
        private IPduManager pduManager;
        private int frameCount = 0;
        private float lastUpdateTime = 0f;
        private int lastPointCount = 0;

        // ランタイム用リソース
        private Material lineGLMaterial = null; // GL.LINES 用マテリアル
        private LineRenderer[] lineRenderersPool = null;
        private ParticleSystem rayParticleSystemInstance = null;
        private ParticleSystem.Particle[] rayParticles = null;
        
        public enum VisualizationMode
        {
            ParticleSystem,
            Gizmos,
            DebugRays
        }
        
        void Start()
        {
            Debug.Log("Drone3DLiDARVisualizer Start() called");

            // ParticleSystemモードの初期化
            if (visualizationMode == VisualizationMode.ParticleSystem)
            {
                InitializeParticleSystem();
            }
            Debug.Log("Rendering camera: " + Camera.main.name);            
            // Ray 描画用リソースを初期化
            InitializeRayResources();
        }
        
        private void InitializeParticleSystem()
        {
            // ParticleSystemコンポーネントを追加
            particleSystem = gameObject.GetComponent<ParticleSystem>();
            if (particleSystem == null)
            {
                particleSystem = gameObject.AddComponent<ParticleSystem>();
            }

            // パーティクルシステムの設定
            var main = particleSystem.main;
            main.loop = false;
            main.playOnAwake = false;
            main.maxParticles = 100000; // 最大ポイント数
            // 世界座標でパーティクル位置を扱う（pointCloudはワールド座標でセットしているため）
            main.simulationSpace = ParticleSystemSimulationSpace.World;
            
            // Emissionを無効化（手動で制御）
            var emission = particleSystem.emission;
            emission.enabled = false;
            
            // Shapeを無効化
            var shape = particleSystem.shape;
            shape.enabled = false;
            
            // パーティクルの見た目
            var renderer = particleSystem.GetComponent<ParticleSystemRenderer>();
            renderer.renderMode = ParticleSystemRenderMode.Billboard;
            if (particleMaterial != null)
            {
                Debug.Log("Using custom particle material for LiDAR visualization");
                renderer.material = particleMaterial;
            }
            else
            {
                Debug.Log("Using default particle material for LiDAR visualization");
                Shader shader = Shader.Find("Particles/Standard Unlit");
                if (shader == null)
                {
                    Debug.Log("Default particle shader not found, trying alternative");
                    // URP/HDRP の場合に標準のパーティクルシェーダが無いことがあるため代替を探す
                    shader = Shader.Find("Particles/Unlit");
                }
                renderer.material = new Material(shader);
                renderer.renderMode = ParticleSystemRenderMode.Billboard;
                renderer.sortingLayerName = "Default";
                renderer.sortingOrder = 0;
                renderer.enabled = true;
                Debug.Log($"Default particle shader used: {shader?.name}");
            }
            // 明示的に再生しておく
            particleSystem.Play();
            if (showDebugInfo)
            {
                Debug.Log($"ParticleSystem initialized for 3D LiDAR visualization (material shader: {renderer.material?.shader?.name})");
            }
        }
        
        /// <summary>
        /// PDUマネージャーを設定
        /// </summary>
        public void SetPduManager(IPduManager manager)
        {
            this.pduManager = manager;
        }
        
        /// <summary>
        /// 外部から呼び出される更新メソッド
        /// </summary>
        public void UpdateVisualization()
        {
            // 更新レート制限
            if (targetUpdateRate > 0)
            {
                float interval = 1f / targetUpdateRate;
                if (Time.time - lastUpdateTime < interval)
                {
                    if (showDebugInfo && frameCount % 60 == 0)
                    {
                        Debug.Log("Skipping update to maintain target update rate.");
                    }
                    return;
                }
                lastUpdateTime = Time.time;
            }
            
            // LiDARデータを取得
            if (pduManager == null)
            {
                if (showDebugInfo && frameCount % 60 == 0)
                {
                    Debug.LogWarning("PDU Manager not set. Call SetPduManager() first.");
                }
                frameCount++;
                return; 
            }
            
            try
            {
                // PointCloud2データを読み取り
                Debug.Log("Reading LiDAR PDU data");
                IPdu pdu = pduManager.ReadPdu(robotName, "lidar_points");

                if (pdu == null)
                {
                    if (showDebugInfo && frameCount % 60 == 0)
                    {
                        Debug.LogWarning($"PDU not found: {robotName}/lidar_points");
                    }
                    frameCount++;
                    Debug.Log("PDU is null, cannot update visualization.");
                    return;
                }

                var pointCloud2 = new PointCloud2(pdu);

                
                // ポイントクラウドをパース
                ParsePointCloud(pointCloud2);
                
                // 可視化を更新
                switch (visualizationMode)
                {
                    case VisualizationMode.ParticleSystem:
                        Debug.Log("Updating Particle System Visualization");
                        UpdateParticleVisualization();
                        break;
                    case VisualizationMode.Gizmos:
                        // GizmosはOnDrawGizmosで描画されるためここでは何もしない
                        break;
                    case VisualizationMode.DebugRays:
                        UpdateDebugRayVisualization();
                        break;
                }
                
                frameCount++;
            }
            catch (Exception e)
            {
                if (showDebugInfo)
                {
                    Debug.LogError($"Error updating LiDAR visualization: {e.Message}");
                }
            }
        }
        
        private void ParsePointCloud(PointCloud2 pointCloud2)
        {
            int height = (int)pointCloud2.height;
            int width = (int)pointCloud2.width;
            int totalPoints = height * width;
            uint pointStep = pointCloud2.point_step;
            
            if (pointCloud == null || pointCloud.Length != totalPoints)
            {
                pointCloud = new Vector3[totalPoints];
            }
            
            byte[] data = pointCloud2.data;
            int validPoints = 0;
            
            for (int i = 0; i < totalPoints; i++)
            {
                int offset = i * (int)pointStep;
                
                // XYZ座標を読み取り (FLOAT32, little-endian)
                float x = BitConverter.ToSingle(data, offset);
                float y = BitConverter.ToSingle(data, offset + 4);
                float z = BitConverter.ToSingle(data, offset + 8);
                // float intensity = BitConverter.ToSingle(data, offset + 12);
                
                // ROS座標系からUnity座標系に変換
                // ROS: X=前, Y=左, Z=上
                // Unity: X=右, Y=上, Z=前
                Vector3 point = new Vector3(
                    -y,  // ROS Y -> Unity X (反転)
                    z,   // ROS Z -> Unity Y
                    x    // ROS X -> Unity Z
                );
                
                // フィルタリング
                float distance = point.magnitude;
                if (distance < minDistance || distance > maxDistance)
                {
                    continue;
                }
                
                if (enableZFilter && (point.y < zMin || point.y > zMax))
                {
                    continue;
                }
                
                pointCloud[validPoints] = point;
                validPoints++;
            }
            
            lastPointCount = validPoints;
            
            // 配列をリサイズ（有効なポイントのみ）
            if (validPoints < totalPoints)
            {
                Array.Resize(ref pointCloud, validPoints);
            }
            
            if (showDebugInfo && frameCount % 60 == 0)
            {
                Debug.Log($"LiDAR Points: {validPoints}/{totalPoints} " +
                         $"(filtered: {totalPoints - validPoints})");
            }
        }
        
        private void UpdateParticleVisualization()
        {
            if (showDebugInfo) Debug.Log("Updating Particle Visualization");
            if (particleSystem == null || pointCloud == null || pointCloud.Length == 0)
            {
                if (showDebugInfo) Debug.LogWarning("ParticleSystem or pointCloud not initialized or empty.");
                return;
            }
            
            // パーティクル配列を準備
            if (particles == null || particles.Length < pointCloud.Length)
            {
                particles = new ParticleSystem.Particle[pointCloud.Length];
            }

            if (showDebugInfo) Debug.Log($"Prepared particles array of size: {particles.Length}");
            // 各ポイントをパーティクルに変換
            for (int i = 0; i < pointCloud.Length; i++)
            {
                Vector3 point = pointCloud[i];
                
                // 色を決定（距離ベース）
                Color color = Color.white;
                if (colorByDistance)
                {
                    float distance = point.magnitude;
                    float t = Mathf.Clamp01(distance / maxDisplayDistance);
                    color = Color.Lerp(nearColor, farColor, t);
                }
                
                particles[i].position = point;
                particles[i].startColor = color;
                particles[i].startSize = particleSize;
                particles[i].remainingLifetime = 1f;
                particles[i].startLifetime = 1f;
            }
            
            Debug.Log($"Setting {pointCloud.Length} particles in ParticleSystem");
            // パーティクルシステムに設定
            particleSystem.SetParticles(particles, pointCloud.Length);
            // 安全のため再生を確認
            particleSystem.Play();
            var rend = particleSystem.GetComponent<ParticleSystemRenderer>();
            rend.enabled = true;
            if (showDebugInfo)
            {
                Debug.Log($"Updated particles: {pointCloud.Length} (material shader: {rend.material?.shader?.name})");
                Camera cam = Camera.main;
                if (cam != null && pointCloud.Length > 0)
                {
                    Vector3 sample = pointCloud[0];
                    Vector3 viewport = cam.WorldToViewportPoint(sample);
                    float dist = Vector3.Distance(cam.transform.position, sample);
                    Debug.Log($"SamplePoint[0]: pos={sample}, viewport={viewport}, dist={dist}");
                    int limit = Math.Min(pointCloud.Length, 5);
                    for (int k = 0; k < limit; k++)
                    {
                        Vector3 p = pointCloud[k];
                        Vector3 vp = cam.WorldToViewportPoint(p);
                        bool inFrustum = (vp.z > 0f && vp.x >= 0f && vp.x <= 1f && vp.y >= 0f && vp.y <= 1f);
                        Debug.Log($"Pt{k}: vp={vp}, inFrustum={inFrustum}");
                    }
                }
                else
                {
                    Debug.LogWarning("Camera.main is null or no points available.");
                }
            }

            // Ray の可視化も更新（オプション）
            if (showRays)
            {
                UpdateRayVisualization();
            }
        }

        /// <summary>
        /// Ray 描画用リソースの初期化（前にあるリソースはクリアする）
        /// </summary>
        public void InitializeRayResources()
        {
            // 常に古いリソースをクリーンにする（モード切替時の残骸防止）
            ClearRayResources();

            if (!showRays) return;
            int poolSize = Math.Max(0, maxRenderedRays / Math.Max(1, raySampling));
            poolSize = Math.Min(poolSize, maxRenderedRays);

            switch (rayVisualization)
            {
                case RayVisualizationMethod.LineRenderer:
                    // LineRenderer プールを作成
                    if (lineRenderersPool == null || lineRenderersPool.Length != poolSize)
                    {
                        lineRenderersPool = new LineRenderer[poolSize];
                        for (int i = 0; i < poolSize; i++)
                        {
                            var go = new GameObject($"LiDAR_Line_{i}");
                            go.transform.parent = this.transform;
                            var lr = go.AddComponent<LineRenderer>();
                            // マテリアル
                            if (rayMaterial != null)
                            {
                                lr.material = rayMaterial;
                            }
                            else
                            {
                                Shader shader = Shader.Find("Unlit/Color") ?? Shader.Find("Sprites/Default") ?? Shader.Find("Particles/Standard Unlit");
                                lr.material = new Material(shader);
                            }
                            lr.positionCount = 2;
                            lr.useWorldSpace = true;
                            lr.startWidth = lineWidth;
                            lr.endWidth = lineWidth;
                            lr.loop = false;
                            lr.enabled = false;
                            lr.numCapVertices = 0;
                            // 色を設定（モード別カラーフィールドを使用）
                            lr.startColor = lineRendererColor;
                            lr.endColor = lineRendererColor;
                            try
                            {
                                lr.material.color = lineRendererColor;
                                lr.material.SetColor("_Color", lineRendererColor);
                            }
                            catch (Exception)
                            {
                                // 無視
                            }
                            lineRenderersPool[i] = lr;
                        }
                    }
                    break;
                case RayVisualizationMethod.GLLines:
                    CreateLineMaterial();
                    break;
                case RayVisualizationMethod.ParticleBeams:
                    // ParticleSystem を生成（プリファブがあればそれを使用）
                    if (rayParticleSystemPrefab != null)
                    {
                        if (rayParticleSystemInstance == null)
                        {
                            rayParticleSystemInstance = Instantiate(rayParticleSystemPrefab, this.transform);
                            var main = rayParticleSystemInstance.main;
                            main.playOnAwake = false;
                        }
                    }
                    else
                    {
                        if (rayParticleSystemInstance == null)
                        {
                            var go = new GameObject("LiDAR_RayParticles");
                            go.transform.parent = this.transform;
                            rayParticleSystemInstance = go.AddComponent<ParticleSystem>();
                            var main = rayParticleSystemInstance.main;
                            main.playOnAwake = false;
                            main.simulationSpace = ParticleSystemSimulationSpace.World;
                            main.maxParticles = Math.Max(1, poolSize);
                            // 小さな固定サイズで描画（ビームの中点を点で示す）
                            main.startSize = 0.02f;
                            main.startLifetime = 0.05f;
                            var renderer = rayParticleSystemInstance.GetComponent<ParticleSystemRenderer>();
                            renderer.renderMode = ParticleSystemRenderMode.Billboard;
                            // マテリアルのフォールバックを確実に行う
                            if (rayMaterial != null)
                            {
                                renderer.material = rayMaterial;
                            }
                            else
                            {
                                Shader shader = Shader.Find("Particles/Standard Unlit") ?? Shader.Find("Universal Render Pipeline/Particles/Unlit") ?? Shader.Find("Unlit/Color") ?? Shader.Find("Sprites/Default");
                                renderer.material = new Material(shader);
                                try { renderer.material.SetColor("_Color", particleBeamsColor); } catch (Exception) { }
                            }
                        }
                    }
                    rayParticles = new ParticleSystem.Particle[Math.Max(1, poolSize)];
                    break;
            }
        }

        /// <summary>
        /// 既存の描画リソースを破棄／無効化する（モード切替や非表示時に使用）
        /// </summary>
        public void ClearRayResources()
        {
            // Destroy GL material
            if (lineGLMaterial != null)
            {
                Destroy(lineGLMaterial);
                lineGLMaterial = null;
            }
            // Destroy LineRenderers
            if (lineRenderersPool != null)
            {
                foreach (var lr in lineRenderersPool)
                {
                    if (lr != null)
                    {
                        Destroy(lr.gameObject);
                    }
                }
                lineRenderersPool = null;
            }
            // Destroy or clear particle beams
            if (rayParticleSystemInstance != null)
            {
                try
                {
                    rayParticleSystemInstance.Clear();
                    Destroy(rayParticleSystemInstance.gameObject);
                }
                catch (Exception) { }
                rayParticleSystemInstance = null;
                rayParticles = null;
            }
        }

        private void CreateLineMaterial()
        {
            if (lineGLMaterial != null) return;
            Shader shader = Shader.Find("Hidden/Internal-Colored") ?? Shader.Find("Unlit/Color") ?? Shader.Find("Sprites/Default");
            lineGLMaterial = new Material(shader)
            {
                hideFlags = HideFlags.HideAndDontSave
            };
            lineGLMaterial.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
            lineGLMaterial.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
            lineGLMaterial.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);
            lineGLMaterial.SetInt("_ZWrite", 0);
        }

        private void UpdateRayVisualization()
        {
            if (!showRays)
            {
                // Ensure existing resources are disabled when showRays becomes false
                if (lineRenderersPool != null)
                {
                    foreach (var lr in lineRenderersPool)
                    {
                        if (lr != null) lr.enabled = false;
                    }
                }
                if (rayParticleSystemInstance != null) rayParticleSystemInstance.Clear();
                return;
            }
            if (pointCloud == null || pointCloud.Length == 0) return;

            // 動的サンプリング: pointCloud の長さが maxRenderedRays を超える場合は step を自動調整
            int step = Math.Max(1, raySampling);
            if (pointCloud.Length > maxRenderedRays)
            {
                step = Mathf.CeilToInt((float)pointCloud.Length / maxRenderedRays);
            }

            int sampled = 0;
            Vector3 origin = transform.position;

            switch (rayVisualization)
            {
                case RayVisualizationMethod.LineRenderer:
                    if (lineRenderersPool == null) InitializeRayResources();
                    int j = 0;
                    for (int i = 0; i < pointCloud.Length && j < lineRenderersPool.Length && sampled < maxRenderedRays; i += step)
                    {
                        var lr = lineRenderersPool[j];
                        lr.enabled = true;
                        lr.SetPosition(0, origin);
                        lr.SetPosition(1, pointCloud[i]);
                        // 線の色を設定（モード別カラーを使用）
                        Gradient g = new Gradient();
                        g.SetKeys(new GradientColorKey[] { new GradientColorKey(lineRendererColor, 0f), new GradientColorKey(lineRendererColor, 1f) }, new GradientAlphaKey[] { new GradientAlphaKey(1f, 0f), new GradientAlphaKey(1f, 1f) });
                        lr.colorGradient = g;
                        lr.startWidth = lineWidth;
                        lr.endWidth = lineWidth;
                        // マテリアル色も更新（すでに存在する LineRenderer に反映）
                        try { lr.material.SetColor("_Color", lineRendererColor); } catch (Exception) { }
                        j++;
                        sampled++;
                    }
                    // 残りは無効化
                    for (int k = j; k < lineRenderersPool.Length; k++)
                    {
                        if (lineRenderersPool[k] != null)
                        {
                            lineRenderersPool[k].enabled = false;
                        }
                    }
                    break;
                case RayVisualizationMethod.ParticleBeams:
                    if (rayParticleSystemInstance == null) InitializeRayResources();
                    int pi = 0;
                    for (int i = 0; i < pointCloud.Length && pi < rayParticles.Length && sampled < maxRenderedRays; i += step)
                    {
                        Vector3 p = pointCloud[i];
                        float dist = Vector3.Distance(origin, p);
                        // 中間点に小さなパーティクルを配置し、色はモード別カラーを使用
                        rayParticles[pi].position = Vector3.Lerp(origin, p, 0.5f);
                        rayParticles[pi].startColor = particleBeamsColor;
                        rayParticles[pi].startSize = Mathf.Clamp(0.02f, 0.01f, 0.05f);
                        rayParticles[pi].startLifetime = 0.05f;
                        rayParticles[pi].remainingLifetime = 0.05f;
                        pi++;
                        sampled++;
                    }
                    if (rayParticleSystemInstance != null)
                    {
                        rayParticleSystemInstance.SetParticles(rayParticles, pi);
                    }
                    break;
                case RayVisualizationMethod.GLLines:
                    // 描画は OnRenderObject で行う（GL で色は緑に統一）
                    break;
            }
        }

        void OnRenderObject()
        {
            if (!showRays) return;
            if (rayVisualization != RayVisualizationMethod.GLLines) return;
            if (pointCloud == null || pointCloud.Length == 0) return;

            CreateLineMaterial();
            if (lineGLMaterial == null) return;
            lineGLMaterial.SetPass(0);

            GL.PushMatrix();
            GL.Begin(GL.LINES);
            int step = Math.Max(1, raySampling);
            Vector3 origin = transform.position;
            int count = Math.Min(pointCloud.Length, maxRenderedRays);
            for (int i = 0; i < count; i += step)
            {
                Vector3 p = pointCloud[i];
                float dist = Vector3.Distance(origin, p);
                // 線の色をモード別カラーに変更
                GL.Color(glLinesColor);
                GL.Vertex(origin);
                GL.Vertex(p);
            }
            GL.End();
            GL.PopMatrix();
        }

        void OnDisable()
        {
            if (lineGLMaterial != null)
            {
                Destroy(lineGLMaterial);
                lineGLMaterial = null;
            }
            if (lineRenderersPool != null)
            {
                foreach (var lr in lineRenderersPool)
                {
                    if (lr != null)
                    {
                        Destroy(lr.gameObject);
                    }
                }
                lineRenderersPool = null;
            }
        }
        
        private void UpdateDebugRayVisualization()
        {
            if (pointCloud == null)
            {
                return;
            }
            
            Vector3 origin = transform.position;
            
            for (int i = 0; i < pointCloud.Length; i++)
            {
                Vector3 point = pointCloud[i];
                Vector3 direction = point - origin;
                float distance = direction.magnitude;
                
                // 色を決定
                Color color = colorByDistance ? 
                    Color.Lerp(nearColor, farColor, distance / maxDisplayDistance) : 
                    Color.red;
                
                Debug.DrawRay(origin, direction, color, 0.05f, false);
            }
        }
        
        void OnDrawGizmos()
        {
            if (visualizationMode != VisualizationMode.Gizmos || pointCloud == null)
            {
                return;
            }
            
            for (int i = 0; i < pointCloud.Length; i++)
            {
                Vector3 point = pointCloud[i];
                float distance = Vector3.Distance(point, transform.position);
                
                // 色を決定
                if (colorByDistance)
                {
                    float t = Mathf.Clamp01(distance / maxDisplayDistance);
                    Gizmos.color = Color.Lerp(nearColor, farColor, t);
                }
                else
                {
                    Gizmos.color = Color.red;
                }
                
                Gizmos.DrawSphere(point, particleSize);
            }
        }
        
        void Update()
        {
            // 自動更新（pduManagerが設定されている場合）
            if (pduManager != null)
            {
                UpdateVisualization();
            }
            else
            {
                if (showDebugInfo && frameCount % 300 == 0)
                {
                    Debug.LogWarning("PDU Manager not set. Visualization not updated.");
                }
            }
        }

        /// <summary>
        /// Enable/disable ray visualization and ensure resources are cleaned up immediately when disabled.
        /// </summary>
        public void SetShowRays(bool enable)
        {
            showRays = enable;
            if (!showRays)
            {
                // Disable existing line renderers and clear particles immediately so they disappear from view
                if (lineRenderersPool != null)
                {
                    foreach (var lr in lineRenderersPool)
                    {
                        if (lr != null) lr.enabled = false;
                    }
                }
                if (rayParticleSystemInstance != null) rayParticleSystemInstance.Clear();
                // Also remove GL material so OnRenderObject won't draw
                if (lineGLMaterial != null)
                {
                    Destroy(lineGLMaterial);
                    lineGLMaterial = null;
                }
            }
            else
            {
                InitializeRayResources();
            }
        }

        /// <summary>
        /// Set the ray color for the currently selected visualization mode and apply it immediately to active resources.
        /// </summary>
        public void SetColorForCurrentRayMode(Color color)
        {
            switch (rayVisualization)
            {
                case RayVisualizationMethod.LineRenderer:
                    lineRendererColor = color;
                    if (lineRenderersPool != null)
                    {
                        foreach (var lr in lineRenderersPool)
                        {
                            if (lr == null) continue;
                            try { lr.material.SetColor("_Color", lineRendererColor); } catch (Exception) { }
                            Gradient g = new Gradient();
                            g.SetKeys(new GradientColorKey[] { new GradientColorKey(lineRendererColor, 0f), new GradientColorKey(lineRendererColor, 1f) }, new GradientAlphaKey[] { new GradientAlphaKey(1f, 0f), new GradientAlphaKey(1f, 1f) });
                            lr.colorGradient = g;
                        }
                    }
                    break;
                case RayVisualizationMethod.GLLines:
                    glLinesColor = color;
                    break;
                case RayVisualizationMethod.ParticleBeams:
                    particleBeamsColor = color;
                    if (rayParticleSystemInstance != null)
                    {
                        var renderer = rayParticleSystemInstance.GetComponent<ParticleSystemRenderer>();
                        try { renderer.material.SetColor("_Color", particleBeamsColor); } catch (Exception) { }
                    }
                    break;
            }
        }

        /// <summary>
        /// Public wrapper to refresh ray visualization (reinitialize resources and update) — safe to call from UI.
        /// </summary>
        public void RefreshRayVisualization()
        {
            InitializeRayResources();
            UpdateRayVisualization();
        }

        /// <summary>
        /// Set the particle size and immediately update the particle visualization.
        /// </summary>
        public void SetParticleSize(float size)
        {
            particleSize = size;
            // If particle visualization is active, update immediately
            if (visualizationMode == VisualizationMode.ParticleSystem && particleSystem != null)
            {
                UpdateParticleVisualization();
            }
        }
        
        /// <summary>
        /// 統計情報を取得
        /// </summary>
        public string GetStats()
        {
            if (pointCloud == null)
            {
                var pduStateEmpty = (pduManager == null) ? "PDU: not set" : "PDU: set";
                return $"No data, {pduStateEmpty}";
            }
            var pduState = (pduManager == null) ? "PDU: not set" : "PDU: set";
            return $"Points: {lastPointCount}, " +
                   $"Mode: {visualizationMode}, " +
                   $"{pduState}, " +
                   $"FPS: {1f / Time.deltaTime:F1}";
        }

        /// <summary>
        /// Gets the ray color for the currently selected visualization mode.
        /// </summary>
        public Color GetColorForCurrentRayMode()
        {
            switch (rayVisualization)
            {
                case RayVisualizationMethod.LineRenderer:
                    return lineRendererColor;
                case RayVisualizationMethod.GLLines:
                    return glLinesColor;
                case RayVisualizationMethod.ParticleBeams:
                    return particleBeamsColor;
                default:
                    return Color.white;
            }
        }
    }
}
