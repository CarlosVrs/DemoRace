using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu(fileName = "New Curve", menuName = "Curves")]
public class Curve : ScriptableObject
{
    public AnimationCurve curve;
    public float factor;
}
