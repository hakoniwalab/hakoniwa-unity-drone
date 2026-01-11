using UnityEngine;
using Oculus.Interaction.Input;

public class HandGrabControllerDisplay : MonoBehaviour
{
    [Header("Hand References")]
    [SerializeField] private Transform leftHand;
    [SerializeField] private Transform rightHand;

    [Header("Display Settings")]
    [SerializeField] private float smoothSpeed = 8f;
    [SerializeField] private Vector3 offset = Vector3.zero;
    [SerializeField] private Transform guideLabel;

    private Renderer[] renderers;
    private float alpha = 1f; // ← 常に表示

    void Start()
    {
        renderers = GetComponentsInChildren<Renderer>();
        SetAlpha(1f); // 最初から表示
    }

    void Update()
    {
        if (leftHand == null || rightHand == null)
            return;

        // 両手の中点へ追従
        Vector3 mid = (leftHand.position + rightHand.position) / 2f;
        Vector3 handDir = (rightHand.position - leftHand.position).normalized;
        Quaternion targetRot = Quaternion.LookRotation(handDir, Vector3.up);

        transform.position = Vector3.Lerp(transform.position, mid + offset, Time.deltaTime * smoothSpeed);
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRot, Time.deltaTime * smoothSpeed);

        // 常にアルファ＝1で表示
        SetAlpha(1f);
    }

    private void SetAlpha(float a)
    {
        foreach (var r in renderers)
        {
            if (r.material.HasProperty("_Color"))
            {
                Color c = r.material.color;
                c.a = a;
                r.material.color = c;
            }
        }

        // TextMeshにも反映
        if (guideLabel != null)
        {
            var tm = guideLabel.GetComponent<TextMesh>();
            if (tm != null)
            {
                Color c = tm.color;
                c.a = a;
                tm.color = c;
            }

            // 演出（上下揺れ）
            guideLabel.localPosition = new Vector3(0, 0.05f + Mathf.Sin(Time.time * 2f) * 0.01f, 0);
        }
    }
}
