using System.Collections;
using System;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;


namespace Unity.Ros.Manipulator
{
    public class Kinematics : MonoBehaviour
    {
        public Kinematics() {}

        public GameObject manipulator;
        private List<ArticulationBody> jointChain;
        
        private readonly Vector vectorZ = new(0, 0, 1);
        private readonly float d1 = 0.101f, d2 = 0.25f, d3 = 0.25f, d5 = 0.096f, d6 = 0.066f;
        private Matrix4x4 A0, A1, A2, A3, A4, A5, A6, T00, T01, T02, T03, T04, T05, T06;        
        private List<Matrix4x4> transformationsMatrices;
        
        private void Start()
        {
            jointChain = new List<ArticulationBody>();

            foreach (ArticulationBody joint in manipulator.GetComponentsInChildren<ArticulationBody>())
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

        public List<Matrix4x4> GetFK(float[] q = null)
        {
            if (q == null) q = GetQ();
            A0 = Matrix4x4.identity;

            A1 = new Matrix4x4(new Vector4(Mathf.Cos(q[0]), 0, -Mathf.Sin(q[0]), 0),
                                new Vector4(Mathf.Sin(q[0]), 0, Mathf.Cos(q[0]), 0),
                                new Vector4(0, -1, 0, d1),
                                new Vector4(0, 0, 0, 1)).transpose;


            A2 = new Matrix4x4(new Vector4(Mathf.Cos(q[1]), -Mathf.Sin(q[1]), 0, d2 * Mathf.Cos(q[1])),
                                new Vector4(Mathf.Sin(q[1]), Mathf.Cos(q[1]), 0, d2 * Mathf.Sin(q[1])),
                                new Vector4(0, 0, 1, 0),
                                new Vector4(0, 0, 0, 1)).transpose;


            A3 = new Matrix4x4(new Vector4(Mathf.Cos(q[2]), -Mathf.Sin(q[2]), 0, d3 * Mathf.Cos(q[2])),
                                new Vector4(Mathf.Sin(q[2]), Mathf.Cos(q[2]), 0, d3 * Mathf.Sin(q[2])),
                                new Vector4(0, 0, 1, 0),
                                new Vector4(0, 0, 0, 1)).transpose;


            A4 = new Matrix4x4(new Vector4(Mathf.Sin(q[3]), 0, Mathf.Cos(q[3]), 0),
                                new Vector4(-Mathf.Cos(q[3]), 0, Mathf.Sin(q[3]), 0),
                                new Vector4(0, -1, 0, 0),
                                new Vector4(0, 0, 0, 1)).transpose;


            A5 = new Matrix4x4(new Vector4(-Mathf.Sin(q[4]), 0, Mathf.Cos(q[4]), 0),
                                new Vector4(Mathf.Cos(q[4]), 0, Mathf.Sin(q[4]), 0),
                                new Vector4(0, 1, 0, d5),
                                new Vector4(0, 0, 0, 1)).transpose;


            A6 = new Matrix4x4(new Vector4(-Mathf.Sin(q[5]), -Mathf.Cos(q[5]), 0, -d6 * Mathf.Sin(q[5])),
                                new Vector4(Mathf.Cos(q[5]), -Mathf.Sin(q[5]), 0, d6 * Mathf.Cos(q[5])),
                                new Vector4(0, 0, 1, 0),
                                new Vector4(0, 0, 0, 1)).transpose;


            T00 = A0;
            T01 = T00 * A1;
            T02 = T01 * A2;
            T03 = T02 * A3;
            T04 = T03 * A4;
            T05 = T04 * A5;
            T06 = T05 * A6;

            transformationsMatrices = new List<Matrix4x4>
            {
                T00,
                T01,
                T02,
                T03,
                T04,
                T05,
                T06
            };

            return transformationsMatrices;
        }

        public IterationResult GetIK(Vector I_r_IE, RotationMatrix C_IE_des, Matrix alpha, float lambda = 0.001f, int max_it = 100, float tol = 0.1f, float[] q_0 = null)
        {
            float[] q;
            if (q_0 == null)
                q = new float[6];
            else
            {
                if (q_0.Length != 6)
                {
                    throw new Exception("Invalid initial q passed into inverse kinematics");
                }
                q = q_0;
            }
            
            int it = 0;
            var dxe = new Matrix(new double[6, 1]);
            bool loosendUpOnce = false;
            IterationResult result = new IterationResult();

            float[] bestQ = q;
            float bestNorm = (float)Double.MaxValue;

            while ((it == 0 || dxe.Norm() > tol) && it < max_it)
            {
                
                GetJacobians(q, out Matrix J_P, out Matrix J_R, out Vector I_r_current, out RotationMatrix R_current);

                Matrix J = Matrix.Stack(J_P, J_R);
                Matrix J_pseudo = J.GetDampedPseudoInverse(lambda);

                Vector dr = I_r_IE - I_r_current;

                Vector dphi = new RotationMatrix((C_IE_des * R_current.Transpose()).matrix).ToAngleAxis();

                dxe = dr.ToMatrix().Stack(dphi.ToMatrix());

                var qd = (alpha * J_pseudo * dxe).ToVectorArray();

                for (int i = 0; i < q.Length; i++)
                {
                    q[i] += (float)qd[i];
                }

                if (it == max_it - 1 && !loosendUpOnce)
                {
                    result.NumberOfIterationsPerformed += max_it / 2;
                    it /= 2;
                    tol *= 10;
                    loosendUpOnce = true;
                    result.DidLoosenUpTolerance = true;
                }

                if (it == max_it - 1 && loosendUpOnce)
                {
                    result.DidConverge = false;
                }
                
                
                if (dxe.Norm() < bestNorm)
                {
                    bestQ = q;
                }
                
                it++;
            }


            result.Q = bestQ;
            result.NumberOfIterationsPerformed += it;

            return result;
        }

        #region GetJacobians

        /// <summary>
        /// Takes the current q vector as an inpute and returns the geometric Position Jacobian J_P, the geometric Rotation Jacobian J_R, the current task-space position r_IE and the current task-space end-effector rotatoin matrix
        /// <para>All are expressed in the I-Frame</para>
        /// </summary>
        /// <param name="q"></param>
        /// <param name="J_P"></param>
        /// <param name="J_R"></param>
        /// <param name="I_r_IE_current"></param>
        /// <param name="I_R_E_current"></param>
        private void GetJacobians(float[] q, out Matrix J_P, out Matrix J_R, out Vector I_r_IE_current, out RotationMatrix I_R_E_current)
        {
            List<Matrix4x4> T_I_k = GetFK(q);
            
            List<RotationMatrix> R_Ik = new List<RotationMatrix>();
            List<Vector> r_Ik = new List<Vector>();
            foreach (var T in T_I_k)
            {
                R_Ik.Add(T.GetRotation());
                r_Ik.Add(new Vector(T[0, 3], T[1, 3], T[2, 3]));
            }

            Matrix4x4 T_IE = T_I_k.Last();
            I_R_E_current = T_IE.GetRotation();
            Vector r_I_IE = new Vector(T_IE[0, 3], T_IE[1, 3], T_IE[2, 3]);


            List<double[]> j = new List<double[]>();
            for (int i = 0; i < 6; i++)
            {                
                j.Add(Vector.Cross(R_Ik[i] * vectorZ, r_I_IE - r_Ik[i]).ToArray());                
            }
            J_P = new Matrix(j.ToArray());

            j = new List<double[]>();
            for (int i = 0; i < 6; i++)
            {
                j.Add((R_Ik[i] * vectorZ).ToArray());
            }
            J_R = new Matrix(j.ToArray());

            I_r_IE_current = r_I_IE;
        }
        #endregion
}
    public class IterationResult
    {
        public float[] Q { get; set; }
        public bool DidLoosenUpTolerance { get; set; } = false;
        public bool DidConverge { get; set; } = true;
        public int NumberOfIterationsPerformed { get; set; }
    }
}

