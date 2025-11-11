using System.Collections;
using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;



namespace Unity.Ros.Quadruped
{
    public class QuadLegKinematics : MonoBehaviour
    {
        public QuadLegKinematics() {}

        public GameObject quadruped;
        private List<ArticulationBody> jointChain;
        
        private readonly float d1 = 0.29785f, d2 = 0.055f, d3 = 0.110945f, d4 = 0.3205f, d5 = 0.025f, d6 = 0.3395f, d3r = -0.110945f;
        private Matrix4x4 A0, A1, A2, A3, T00, T01, T02, T03;        
        private List<Matrix4x4> transformationsMatrices;
        private readonly float pi = Mathf.PI;
        
        private void Start()
        {
            jointChain = new List<ArticulationBody>();

            foreach (ArticulationBody joint in quadruped.GetComponentsInChildren<ArticulationBody>())
            {
                if (joint.jointType == ArticulationJointType.RevoluteJoint)
                    jointChain.Add(joint);
            }
        }

        public float[] GetQ()
        {
            float[] q = new float[6];
            for (int i = 0; i < jointChain.Count; i++)
            {
                q[i] = ((float)Math.Round(jointChain[i].jointPosition[0], 2));
            }
            return q;
        }

        public float[] GetRearLeftQ()
        {
            float[] q = new float[3];
            for (int i = 0; i < jointChain.Count; i++)
            {
                q[i] = (float)Math.Round(jointChain[i].jointPosition[0], 2);
            }
            return q;
        }

        public List<Matrix4x4> GetRearLeftFK(float[] q = null)
        {
            if (q == null) q = GetRearLeftQ();
            A0 = Matrix4x4.identity;

            A1 = new Matrix4x4( new Vector4(0,                             -1,                      0,                             -d1),
                                new Vector4(Mathf.Cos(q[0] - pi / 2),       0,                     -Mathf.Sin(q[0] - pi / 2),       d2),
                                new Vector4(Mathf.Sin(q[0] - pi / 2),       0,                      Mathf.Cos(q[0] - pi / 2),       0),
                                new Vector4(0,                              0,                      0,                              1)).transpose;


            A2 = new Matrix4x4( new Vector4(Mathf.Cos(q[1]),               -Mathf.Sin(q[1]),        0,                              d4 * Mathf.Cos(q[1]) + d5 * Mathf.Sin(q[1])),
                                new Vector4(Mathf.Sin(q[1]),                Mathf.Cos(q[1]),        0,                              d4 * Mathf.Sin(q[1]) - d5 * Mathf.Cos(q[1])),
                                new Vector4(0,                              0,                      1,                              d3),
                                new Vector4(0,                              0,                      0,                              1)).transpose;


            A3 = new Matrix4x4( new Vector4(Mathf.Sin(q[2]),                0,                     -Mathf.Cos(q[2]),                d6 * Mathf.Cos(q[2])),
                                new Vector4(-Mathf.Cos(q[2]),               0,                     -Mathf.Sin(q[2]),                d6 * Mathf.Sin(q[2])),
                                new Vector4(0,                              1,                      0,                              0),
                                new Vector4(0,                              0,                      0,                              1)).transpose;

            T00 = A0;
            T01 = T00 * A1;
            T02 = T01 * A2;
            T03 = T02 * A3;

            transformationsMatrices = new List<Matrix4x4>
            {
                T00,
                T01,
                T02,
                T03,
            };

            return transformationsMatrices;
        }

        public float[] GetLeftIK(Vector3 position)
        {
            float[] q = new float[3];

            float alpha = Mathf.Atan2(-1 * position.z, position.y);
            float beta = Mathf.Atan2(Mathf.Sqrt(position.y * position.y + position.z * position.z - d3 * d3), d3);
            q[0] = beta - alpha;

            float dESquare = position.x * position.x + position.y * position.y + position.z * position.z;
            float d43 = Mathf.Sqrt(d4 * d4 + d3 * d3);
            float cosSigma = (dESquare - (d43 * d43 + d6 * d6 + d3 * d3)) / (2 * d43 * d6);
            float sinSigma = -Mathf.Sqrt(1 - (cosSigma * cosSigma));
            float sigma = Mathf.Atan2(d6 * sinSigma, d6 * cosSigma);
            q[2] = sigma;

            alpha = Mathf.Atan2(-1 * position.x, Mathf.Sqrt(position.y * position.y + position.z * position.z - d3 * d3));
            beta = Mathf.Atan2(d6 * sinSigma, d4 + d6 * cosSigma);
            q[1] = alpha - beta;

            return q;
        }

        public float[] GetRightIK(Vector3 position)
        {
            float[] q = new float[3];

            float alpha = Mathf.Atan2(-1 * position.z, position.y);
            float beta = Mathf.Atan2(Mathf.Sqrt(position.y * position.y + position.z * position.z - d3 * d3), d3r);
            q[0] = beta - alpha;

            float dESquare = position.x * position.x + position.y * position.y + position.z * position.z;
            float d43 = Mathf.Sqrt(d4 * d4 + d3 * d3);
            float cosSigma = (dESquare - (d43 * d43 + d6 * d6 + d3 * d3)) / (2 * d43 * d6);
            float sinSigma = -Mathf.Sqrt(1 - (cosSigma * cosSigma));
            float sigma = Mathf.Atan2(d6 * sinSigma, d6 * cosSigma);
            q[2] = sigma;

            alpha = Mathf.Atan2(-1 * position.x, Mathf.Sqrt(position.y * position.y + position.z * position.z - d3 * d3));
            beta = Mathf.Atan2(d6 * sinSigma, d4 + d6 * cosSigma);
            q[1] = alpha - beta;

            return q;
        }
    }
    
    public class IterationResult
    {
        public float[] Q { get; set; }
        public bool DidLoosenUpTolerance { get; set; } = false;
        public bool DidConverge { get; set; } = true;
        public int NumberOfIterationsPerformed { get; set; }
    }
}

