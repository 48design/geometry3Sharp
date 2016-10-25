using System;
using System.Runtime.InteropServices;
using System.Collections.Generic;
using System.Text;

namespace g3
{
    public class Vector2f
    {
        public float[] v = { 0, 0 };

        public Vector2f() { }
        public Vector2f(float f) { v[0] = v[1] = f; }
        public Vector2f(float x, float y) { v[0] = x; v[1] = y; }
        public Vector2f(float[] v2) { v[0] = v2[0]; v[1] = v2[1]; }
        public Vector2f(Vector2f copy) { v[0] = copy[0]; v[1] = copy[1]; }


        static public Vector2f Zero
        {
            get { return new Vector2f(0.0f, 0.0f); }
        }



        public float x
        {
            get { return v[0]; }
            set { v[0] = value; }
        }
        public float y
        {
            get { return v[1]; }
            set { v[1] = value; }
        }
        public float this[int key]
        {
            get { return v[key]; }
            set { v[key] = value; }
        }


        public float LengthSquared
        {
            get { return v[0] * v[0] + v[1] * v[1]; }
        }
        public float Length
        {
            get { return (float)Math.Sqrt(LengthSquared); }
        }

        public float Normalize()
        {
            float f = Length;
            v[0] /= f; v[1] /= f; v[2] /= f;
            return f;
        }
        public Vector2f Normalized
        {
            get { float f = Length; return new Vector2f(v[0] / f, v[1] / f); }
        }


        public float Dot(Vector2f v2)
        {
            return v[0] * v2[0] + v[1] * v2[1];
        }


        public float Cross(Vector2f v2) {
            return y * v2[1] - y * v2[0];
        }



        public float SquaredDist(Vector2f o) {
            return ((v[0] - o[0]) * (v[0] - o[0]) + (v[1] - o[1]) * (v[1] - o[1]));
        }
        public float Dist(Vector2f o) {
            return (float)Math.Sqrt((v[0] - o[0]) * (v[0] - o[0]) + (v[1] - o[1]) * (v[1] - o[1]));
        }


        public void Set(Vector2f o) {
            v[0] = o[0]; v[1] = o[1];
        }
        public void Set(float fX, float fY) {
            v[0] = fX; v[1] = fY;
        }
        public void Add(Vector2f o) {
            v[0] += o[0]; v[1] += o[1];
        }
        public void Subtract(Vector2f o) {
            v[0] -= o[0]; v[1] -= o[1];
        }


        public static Vector2f operator+( Vector2f a, Vector2f o ) {
            return new Vector2f(a[0] + o[0], a[1] + o[1]); 
        }
        public static Vector2f operator +(Vector2f a, float f) {
            return new Vector2f(a[0] + f, a[1] + f);
        }

        public static Vector2f operator-(Vector2f a, Vector2f o) {
            return new Vector2f(a[0] - o[0], a[1] - o[1]);
        }
        public static Vector2f operator -(Vector2f a, float f) {
            return new Vector2f(a[0] - f, a[1] - f);
        }

        public static Vector2f operator *(Vector2f a, float f) {
            return new Vector2f(a[0] * f, a[1] * f);
        }
        public static Vector2f operator *(float f, Vector2f a) {
            return new Vector2f(a[0] * f, a[1] * f);
        }

        

    }
}