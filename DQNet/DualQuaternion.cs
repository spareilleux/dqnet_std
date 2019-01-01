// --------------------------------------------------------------------------------------------------------------------
// <copyright file="DualQuaternion.cs" company="Stéphane Pareilleux">
// Copyright 2015-2019 Stéphane Pareilleux
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// </copyright>
// <summary>
//   Ported from C to C#:
//   - The C libdq library (See http://www.iri.upc.edu/people/esimo/code/libdq/)
//
//   Inspired from:
//   - "A Beginners Guide to Dual-Quaternions" (See http://wscg.zcu.cz/wscg2012/short/a29-full.pdf)
//   - SharpDX Quaternion class
// </summary>
// --------------------------------------------------------------------------------------------------------------------

using System;
using System.Diagnostics.CodeAnalysis;
using System.Linq;
using System.Runtime.InteropServices;
using System.Numerics;

using Matrix3x3 = Accord.Math.Matrix3x3;
using Matrix4x4 = Accord.Math.Matrix4x4;

using JetBrains.Annotations;

namespace DQNet
{
    /// <inheritdoc cref="IEquatable{DualQuaternion}" />
    /// <summary>
    /// Represents a dual quaternion.
    /// </summary>
    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    [PublicAPI]
    public readonly struct DualQuaternion : IEquatable<DualQuaternion>
    {
        /// <summary>
        /// The tolerance below which a double is considered as equivalent to zero.
        /// </summary>
        public const double ZeroTolerance = 1e-8f;

        #region Fields

        /// <summary>
        /// A dual quaternion with and identity default part and a zero dual part.
        /// </summary>
        public static readonly DualQuaternion Default = new DualQuaternion(Quaternion.Identity, new Quaternion());

        /// <summary>
        /// A dual quaternion with all of its components set to zero.
        /// </summary>
        public static readonly DualQuaternion Zero;

        /// <summary>
        /// A dual quaternion that represents the origin point (X:0, Y:0, Z:0).
        /// </summary>
        public static readonly DualQuaternion OriginPoint = CreateFromPoint(new Vector3());

        /// <summary>
        /// The real part of the dual quaternion.
        /// </summary>
        public readonly Quaternion Real;

        /// <summary>
        /// The dual part of the dual quaternion.
        /// </summary>
        public readonly Quaternion Dual;

        /// <summary>
        /// Number digits used for rounding. 
        /// </summary>
        private static readonly int RoundingDigits = 3;

        #endregion

        #region Constructors and Destructors

        /// <summary>
        /// Initializes a new instance of the <see cref="DualQuaternion"/> struct (From real and dual quaternions, the real gets normalized).
        /// </summary><param name="real">The real.</param>
        /// <param name="dual">The dual.</param>
        public DualQuaternion(
            Quaternion real,
            Quaternion dual)
        {
            Real = Quaternion.Normalize(real);
            Dual = dual;
        }

        /// <inheritdoc />
        /// <summary>
        /// Initializes a new instance of the <see cref="DualQuaternion" /> struct.
        /// </summary><param name="realX">The real x.</param>
        /// <param name="realY">The real y.</param>
        /// <param name="realZ">The real z.</param>
        /// <param name="realW">The real w.</param>
        /// <param name="dualX">The dual x.</param>
        /// <param name="dualY">The dual y.</param>
        /// <param name="dualZ">The dual z.</param>
        /// <param name="dualW">The dual w.</param>
        [SuppressMessage("StyleCop.CSharp.DocumentationRules", "SA1650:ElementDocumentationMustBeSpelledCorrectly", Justification = "Suppression is OK here.")]
        public DualQuaternion(
            float realX, float realY, float realZ, float realW, 
            float dualX, float dualY, float dualZ, float dualW)
            : this(
                new Quaternion(realX, realY, realZ, realW), 
                new Quaternion(dualX, dualY, dualZ, dualW))
        {
        }

        #endregion

        #region Public Properties

        /// <summary>
        /// Gets the rotation from the dual quaternion.
        /// </summary>
        /// <value>A <see cref="Quaternion"/>.</value>
        public Quaternion Rotation => Real;

        /// <summary>
        /// Gets the translation from the dual quaternion.
        /// </summary>
        /// <value>A <see cref="Vector3"/>.</value>
        public Vector3 Translation
        {
            get
            {
                var t = Dual * 2.0f * Quaternion.Conjugate(Real);
                var result = new Vector3(t.X, t.Y, t.Z);

                return result;
            }
        }

        /// <summary>
        /// Gets the length of the dual quaternion.
        /// </summary>
        /// <value>A <see cref="double"/>.</value>
        public double Length
        {
            get
            {
                var result = Quaternion.Dot(Real, Real);

                return result;
            }
        }

        /// <summary>
        /// Gets a boolean indicating whether the dual quaternion is a unit dual quaternion.
        /// </summary>
        public bool IsUnit
        {
            get
            {
                var (realLengthSquared, dualLengthSquared) = GetLengthSquared();

                var result =
                    !(Math.Abs(realLengthSquared - 1) > ZeroTolerance) &&
                    !(Math.Abs(dualLengthSquared) > ZeroTolerance);

                return result;
            }
        }

        #endregion

        #region Indexer

        /// <summary>
        /// Gets a dual quaternion component by index.
        /// </summary>
        /// <param name="index">The index.</param>
        /// <returns>A <see cref="float"/> component of the dual quaternion.</returns>
        public float this[int index]
        {
            get
            {
                switch (index)
                {
                    case 0: return Real.X;
                    case 1: return Real.Y;
                    case 2: return Real.Z;
                    case 3: return Real.W;
                    case 4: return Dual.X;
                    case 5: return Dual.Y;
                    case 6: return Dual.Z;
                    case 7: return Dual.W;
                    default:
                        throw new ArgumentOutOfRangeException(nameof(index), "Indices for DualQuaternion run from 0 to 7, inclusive.");
                }
            }
        }

        #endregion

        #region Operators

        /// <summary>
        /// Adds two dual quaternions.
        /// </summary>
        /// <param name="left">The left <see cref="DualQuaternion"/>.</param>
        /// <param name="right">The right <see cref="DualQuaternion"/>.</param>
        /// <returns>A <see cref="DualQuaternion"/>.</returns>
        public static DualQuaternion operator +(in DualQuaternion left, in DualQuaternion right)
        {
            var result = new DualQuaternion(
                left.Real + right.Real,
                left.Dual + right.Dual);

            return result;
        }

        /// <summary>
        /// Subtracts two dual quaternions.
        /// </summary>
        /// <param name="left">The left <see cref="DualQuaternion"/>.</param>
        /// <param name="right">The right <see cref="DualQuaternion"/>.</param>
        /// <returns>A <see cref="DualQuaternion"/>.</returns>
        public static DualQuaternion operator -(in DualQuaternion left, in DualQuaternion right)
        {
            var result = new DualQuaternion(
                left.Real - right.Real,
                left.Dual - right.Dual);

            return result;
        }

        /// <summary>
        /// Swaps the sign of all the elements in a dual quaternion.
        /// </summary>
        /// <param name="value">The dual quaternion to negate.</param>
        /// <returns>A <see cref="DualQuaternion"/>.</returns>
        public static DualQuaternion operator -(in DualQuaternion value)
        {
            var r = value.Real;
            var d = value.Dual;

            var result = new DualQuaternion(-r.X, -r.Y, -r.Z, -r.W, -d.X, -d.Y, -d.Z, -d.W);

            return result;
        }

        /// <summary>
        /// Applies a scale factor to a dual quaternion.
        /// </summary>
        /// <param name="value">The dual quaternion to scale.</param>
        /// <param name="scale">The scale amount <see cref="float"/>.</param>
        /// <returns>The scaled <see cref="DualQuaternion"/>.</returns>
        public static DualQuaternion operator *(in DualQuaternion value, float scale)
        {
            var result = new DualQuaternion(value.Real * scale, value.Dual * scale);

            return result;
        }

        /// <summary>
        /// Applies a scale factor to a dual quaternion.
        /// </summary>
        /// <param name="scale">The scale amount <see cref="float"/>.</param>
        /// <param name="value">The <see cref="DualQuaternion"/> to scale.</param>
        /// <returns>The scaled <see cref="DualQuaternion"/>.</returns>
        public static DualQuaternion operator *(float scale, in DualQuaternion value)
        {
            var result = new DualQuaternion(value.Real * scale, value.Dual * scale);

            return result;
        }

        /// <summary>
        /// Multiplies a dual quaternion by another.
        /// </summary>
        /// <param name="left">The left <see cref="DualQuaternion"/>.</param>
        /// <param name="right">The right <see cref="DualQuaternion"/></param>
        /// <returns>The product <see cref="DualQuaternion"/>.</returns>
        public static DualQuaternion operator *(in DualQuaternion left, in DualQuaternion right)
        {
            ////  We can decompose the problem into quaternion multiplication:            
            ////  Q = q + εq0
            ////  P = p + εp0
            ////  Q*P = q*p + ε(q*p0 + q0*p) 

            var q = left.Real;
            var p = right.Real;
            var q0 = left.Dual;
            var p0 = right.Dual;

            var result = new DualQuaternion(
                q * p,
                q * p0 + q0 * p);

            return result;
        }

        /// <summary>
        /// Tests for equality between two dual quaternions.
        /// </summary>
        /// <param name="left">The left <see cref="DualQuaternion"/>.</param>
        /// <param name="right">The right <see cref="DualQuaternion"/></param>
        /// <returns><c>true</c> if <paramref name="left"/> has the same value as <paramref name="right"/>; otherwise, <c>false</c>.</returns>
        public static bool operator ==(in DualQuaternion left, in DualQuaternion right)
        {
            return left.Equals(right);
        }

        /// <summary>
        /// Tests for inequality between two dual quaternions.
        /// </summary>
        /// <param name="left">The left <see cref="DualQuaternion"/>.</param>
        /// <param name="right">The right <see cref="DualQuaternion"/></param>
        /// <returns><c>true</c> if <paramref name="left"/> has a different value than <paramref name="right"/>; otherwise, <c>false</c>.</returns>
        public static bool operator !=(in DualQuaternion left, in DualQuaternion right)
        {
            return !left.Equals(right);
        }

        #endregion

        #region Methods

        /// <summary>
        /// Creates a pure rotation dual quaternion.
        /// </summary>
        /// <param name="angle">Angle to rotate.</param>
        /// <param name="axis">Axis <see cref="Vector3"/> to rotate around (normalized vector).</param>
        /// <param name="point">Any <see cref="Vector3"/> point of the axis (to create Plücker coordinates).</param>
        /// <returns>The rotation <see cref="DualQuaternion"/>.</returns>
        public static DualQuaternion CreateFromRotation(
            double angle,
            Vector3 axis,
            Vector3 point)
        {
            // We do cross product with the line point and line vector to get the Plücker coordinates.
            var moment = Vector3.Cross(point, axis);
            var result = CreateRotationPlucker(angle, axis, moment);

            return result;
        }

        /// <summary>
        /// Creates a pure rotation dual quaternion using Plücker coordinates.
        /// </summary>
        /// <param name="angle">The angle to rotate.</param>
        /// <param name="axis">The axis to rotate around (normalized vector).</param>
        /// <param name="moment">Moment of the axis.</param>
        /// <returns>The rotation (Plücker) <see cref="DualQuaternion"/>.</returns>
        public static DualQuaternion CreateRotationPlucker(
            double angle,
            Vector3 axis,
            Vector3 moment)
        {
            var s = Adjust(Math.Sin(angle / 2f));
            var c = Adjust(Math.Cos(angle / 2f));

            var result = new DualQuaternion(
                s * axis.X,
                s * axis.Y,
                s * axis.Z,
                c,
                s * moment.X,
                s * moment.Y,
                s * moment.Z,
                0);

            return result;
        }

        /// <summary>
        /// Creates a pure translation dual unit quaternion (1 + ε½t).
        /// </summary>
        /// <param name="vector">The translation <see cref="Vector3"/> vector.</param>
        /// <returns>The translation <see cref="DualQuaternion"/>.</returns>
        public static DualQuaternion CreateFromTranslation(Vector3 vector)
        {
            return CreateFromTranslation(1, vector);
        }

        /// <summary>
        /// Creates a pure translation dual quaternion (1 + ε½t * amount)
        /// </summary>
        /// <param name="amount">The translation <see cref="double"/> amount</param>
        /// <param name="vector">The translation <see cref="Vector3"/> vector.</param>
        /// <returns>The translation <see cref="DualQuaternion"/>.</returns>
        public static DualQuaternion CreateFromTranslation(
            double amount,
            Vector3 vector)
        {
            var result = new DualQuaternion(
                0,
                0,
                0,
                1.0f,
                (float)(amount * vector.X / 2d),
                (float)(amount * vector.Y / 2d),
                (float)(amount * vector.Z / 2d),
                0);

            return result;
        }

        /// <summary>
        /// Creates a dual quaternion representing a point.
        /// </summary>
        /// <param name="point">The <see cref="Vector3"/> position of the point.</param>
        /// <returns>The point <see cref="DualQuaternion"/>.</returns>
        public static DualQuaternion CreateFromPoint(Vector3 point)
        {
            var result = new DualQuaternion(
                0,
                0,
                0,
                1.0f,
                point.X,
                point.Y,
                point.Z,
                0);

            return result;
        }

        /// <summary>
        /// Creates a dual quaternion representing a line.
        /// </summary><param name="vector">Direction <see cref="Vector3"/> vector of the line.</param>
        /// <param name="point">The <see cref="Vector3"/> point of the line.</param>
        /// <returns>The line <see cref="DualQuaternion"/>.</returns>
        public static DualQuaternion CreateLine(
            Vector3 vector,
            Vector3 point)
        {
            // We do cross product with the line point and line vector to get the Plücker coordinates.
            var s0 = Vector3.Cross(point, vector);
            var result = CreateLinePlucker(vector, s0);

            return result;
        }

        /// <summary>
        /// Creates a dual quaternion representing a line from Plücker coordinates.
        /// </summary>
        /// <param name="vector">Direction <see cref="Vector3"/> vector of the line.</param>
        /// <param name="moment">The <see cref="Vector3"/> moment of the line.</param>
        /// <returns>The line<see cref="DualQuaternion"/>.</returns>
        public static DualQuaternion CreateLinePlucker(
            Vector3 vector,
            Vector3 moment)
        {
            var result = new DualQuaternion(
                vector.X, vector.Y, vector.Z, 0,
                moment.X, moment.Y, moment.Z, 0);

            return result;
        }

        /// <summary>
        /// Creates a unit dual quaternion representing a plane.
        /// </summary>
        /// <param name="normal">Normal <see cref="Vector3"/> of the plane.</param>
        /// <param name="distance">The <see cref="float"/> distance from the origin to the plane.</param>
        /// <returns>The plane <see cref="DualQuaternion"/>.</returns>
        public static DualQuaternion CreatePlane(Vector3 normal, float distance)
        {
            var result = new DualQuaternion(
                normal.X, normal.Y, normal.Z, 0,
                0, 0, 0, distance);

            return result;
        }

        /// <summary>
        /// Creates a pure rotation dual quaternion from a rotation matrix.
        /// </summary>
        /// <param name="rotation">The <see cref="Matrix3x3"/> 3x3 Rotation matrix.</param>
        /// <returns>The rotation <see cref="DualQuaternion"/>.</returns>
        public static DualQuaternion CreateRotationMatrix(Matrix3x3 rotation)
        {
            // B = (rotation - I).(R + I)^{-1}
            var rMinus = rotation - Matrix3x3.Identity;
            var rPlus = rotation + Matrix3x3.Identity;
            rPlus = rPlus.Inverse();
            var b = rMinus * rPlus;

            /*
                      0 -b_z  b_y
                 B = b_z  0  -b_x
                    -b_y b_x   0
                
                 b = { b_x b_y b_z }
                
                 s           = b / ||b||
                 tan(theta/2) = ||b||
             */

            var s = new Vector3(b.V21, b.V02, b.V10);
            var tz = s.Length();

            // Avoid normalizing 0. vectors.
            if (tz > 0)
            {
                s.X = s.X / tz;
                s.Y = s.Y / tz;
                s.Z = s.Z / tz;
            }

            var z2 = Math.Atan(tz);

            // Build the rotational part.
            var sz = Math.Sin(z2);
            var cz = Math.Cos(z2);

            var result =
                new DualQuaternion(
                    (float)(sz * s.X), (float)(sz * s.Y), (float)(sz * s.Z), (float)cz,
                    0, 0, 0, 0);

            return result;
        }

        /// <summary>
        /// Creates a rotation then translation dual quaternion (r + ε½tr).
        /// </summary>
        /// <param name="rotation">The rotation <see cref="Quaternion"/>.</param>
        /// <param name="translation">The translation <see cref="Vector3"/>.</param>
        /// <returns>The translation <see cref="DualQuaternion"/>.</returns>
        public static DualQuaternion CreateRotationThenTranslation(in Quaternion rotation, Vector3 translation)
        {
            var factor = Quaternion.Normalize(rotation) * 0.5f;
            var dual = new Quaternion(translation, 0) * factor;

            var result = new DualQuaternion(factor, dual);

            return result;
        }

        /// <summary>
        /// Calculates the dot product of two dual quaternions.
        /// </summary>
        /// <param name="left">The left <see cref="DualQuaternion"/>.</param>
        /// <param name="right">The right <see cref="DualQuaternion"/>.</param>
        /// <returns>The <see cref="float"/> result of the dot product.</returns>
        public static float Dot(in DualQuaternion left, in DualQuaternion right)
        {
            var result = Quaternion.Dot(left.Real, right.Real);

            return result;
        }

        /// <summary>
        /// Converts the dual quaternion to a unit dual quaternion.
        /// </summary>
        /// <param name="value">The <see cref="DualQuaternion"/> to normalize.</param>
        /// <returns>
        /// The normalized <see cref="DualQuaternion"/>.
        /// </returns>
        public static DualQuaternion Normalize(DualQuaternion value)
        {
            var scale = Adjust(1 / value.Length);
            var result = new DualQuaternion(
                value.Real * scale,
                value.Dual * scale);

            return result;
        }

        /// <summary>
        /// Conjugates a dual quaternion.
        /// </summary>
        /// <param name="value">The <see cref="DualQuaternion"/> to conjugate.</param>
        /// <returns>The conjugated <see cref="DualQuaternion"/>.</returns>
        public static DualQuaternion Conjugate(in DualQuaternion value)
        {
            var result = new DualQuaternion(
                Quaternion.Conjugate(value.Real),
                Quaternion.Conjugate(value.Dual));

            return result;
        }

        /// <summary>
        /// Clifford conjugation transformation of type F1G (Alba Perez notation).
        /// </summary>
        /// <remarks>
        /// f1G : C(V, &lt;, &gt;) −→ C(V, &lt;, &gt;)
        /// A : B −→ ABA
        /// </remarks>
        /// <param name="transformation">The transformation <see cref="DualQuaternion"/>.</param>
        /// <param name="value">The <see cref="DualQuaternion"/> being transformed.</param>
        /// <returns>The resulting <see cref="DualQuaternion"/>.</returns>
        public static DualQuaternion CliffordConjugationTransformationF1G(
            in DualQuaternion transformation,
            in DualQuaternion value)
        {
            var result = transformation * value * transformation;

            return result;
        }

        /// <summary>
        /// Clifford conjugation transformation of type F2G (Alba Perez notation) - This transformation is useful for lines.
        /// </summary>
        /// <remarks>
        /// A : B −→ ABA∗
        /// f2G : C(V, &lt;, &gt;) −→ C(V, &lt;, &gt;)
        /// </remarks>
        /// <param name="value">The <see cref="DualQuaternion"/> being transformed.</param>
        /// <param name="transformation">The <see cref="DualQuaternion"/> representing the transformation.</param>
        /// <returns>The resulting <see cref="DualQuaternion"/>.</returns>
        public static DualQuaternion CliffordConjugationTransformationF2G(
            in DualQuaternion value,
            in DualQuaternion transformation)
        {
            var transformationStar = Conjugate(transformation);
            var result = transformation * value * transformationStar;

            return result;
        }

        /// <summary>
        /// Clifford conjugation transformation of type F3G (Alba Perez notation).
        /// </summary>
        /// <remarks>
        /// f3G : C(V, &lt;, &gt;) −→ C(V, &lt;, &gt;)
        /// A : B −→ AB(a0 + a − E(a^0 + a7))
        /// </remarks>
        /// <param name="value">The <see cref="DualQuaternion"/> being transformed.</param>
        /// <param name="transformation">The <see cref="DualQuaternion"/> representing the transformation.</param>
        /// <returns>The resulting <see cref="DualQuaternion"/>.</returns>
        public static DualQuaternion CliffordConjugationTransformationF3G(
            in DualQuaternion value,
            in DualQuaternion transformation)
        {
            var tr = transformation.Real;
            var td = transformation.Dual;
            var transformationStar =
                new DualQuaternion(
                    tr.X, tr.Y, tr.Z, tr.W,
                    -td.X, -td.Y, -td.Z, -td.W);
            var result = transformation * value * transformationStar;

            return result;
        }

        /// <summary>
        /// Applies a clifford conjugation transformation of type F4G (Alba Perez notation) - This transformation is useful for points.
        /// </summary>
        /// <param name="value">Dual quaternion being transformed. </param>
        /// <param name="transformation">Dual quaternion representing the transformation.</param>
        /// <param name="adjust">Set to true for adjusting the resulting dual quaternion (Defaulted to true).</param>
        /// <returns>The resulting <see cref="DualQuaternion"/>.</returns>
        public static DualQuaternion CliffordConjugationTransformationF4G(
            in DualQuaternion value,
            in DualQuaternion transformation,
            bool adjust = true)
        {
            var tr = transformation.Real;
            var td = transformation.Dual;
            var transformationStar =
                new DualQuaternion(
                    -tr.X, -tr.Y, -tr.Z, tr.W,
                    td.X, td.Y, td.Z, -td.W);
            var result = transformation * value;
            result = result * transformationStar;
            if (adjust) result = result.Adjusted();

            return result;
        }

        /// <summary>
        /// Multiplies a collection of dual quaternion transformations.
        /// </summary>
        /// <param name="values">The collection of dual quaternions.</param>
        /// <returns>The product of all dual quaternions.</returns>
        public static DualQuaternion Multiply(params DualQuaternion[] values)
        {
            if (!values.Any()) throw new ArgumentException("Must define one or more transformations", nameof(values));
            var q = values[values.Length - 1];

            double x = q[0];
            double y = q[1];
            double z = q[2];
            double w = q[3];

            double x0 = q[4];
            double y0 = q[5];
            double z0 = q[6];
            double w0 = q[7];

            for (var i = values.Length - 2; i >= 0; i--)
            {
                Combine(
                    values[i],
                    ref x, ref y, ref z, ref w,
                    ref x0, ref y0, ref z0, ref w0);
            }

            var result = new DualQuaternion(
                (float)x, (float)y, (float)z, (float)w,
                (float)x0, (float)y0, (float)z0, (float)w0);

            return result;
            // ReSharper restore InconsistentNaming
        }

        /// <summary>
        /// Applies a Clifford conjugation transformation of type F4G, then convert the result to a point.
        /// </summary>
        /// <param name="value">The point being transformed.</param>
        /// <param name="transformation">The dual quaternion representing the transformation.</param>
        /// <param name="round">If true, applies rounding to the transformed point coordinates.</param>
        /// <returns>The transformed point.</returns>
        public static Vector3 TransformPoint(
            Vector3 value,
            in DualQuaternion transformation,
            bool round)
        {
            var initialPoint = CreateFromPoint(value);
            var q = CliffordConjugationTransformationF4G(initialPoint, transformation, round);
            var result = q.GetPoint(round);

            return result;
        }

        /// <summary>
        /// Checks to see if a point Q is on the plane P.
        /// </summary>
        /// <param name="point">The point.</param>
        /// <param name="plane">The plane.</param>
        /// <returns>True if the point is on the plane.</returns>
        public static bool IsPointOnPlane(in DualQuaternion point, in DualQuaternion plane)
        {
            var result = Math.Abs(
                plane.Real.X * point.Dual.X +
                plane.Real.Y * point.Dual.Y +
                plane.Real.Z * point.Dual.Z - plane.Dual.Z)
                < ZeroTolerance;

            return result;
        }

        /// <summary>
        /// Compares two dual quaternions.
        /// </summary>
        /// <param name="left">The first dual quaternion.</param>
        /// <param name="right">The second dual quaternion.</param>
        /// <param name="precision">The comparison precision.</param>
        /// <returns>0 if both dual quaternions are equal.</returns>
        public static int Compare(
            in DualQuaternion left,
            in DualQuaternion right,
            double precision = ZeroTolerance)
        {
            // To compensate the rotational ambiguity we see if the 'z' component is the same.
            var ret1 = 0;
            for (var i = 0; i < 8; i++)
            {
                if (Math.Abs(left[i] - right[i]) > precision) ret1++;
            }

            var ret2 = 0;
            for (var i = 0; i < 8; i++)
            {
                if (Math.Abs(left[i] + right[i]) > precision) ret2++;
            }

            var result = Math.Min(ret1, ret2);

            return result;
        }

        /// <summary>
        /// Gets the conjugates.
        /// </summary>
        /// <returns>The conjugate <see cref="DualQuaternion"/>.</returns>
        [Pure]
        public DualQuaternion Conjugate()
        {
            var result = new DualQuaternion(
                Quaternion.Conjugate(Real),
                Quaternion.Conjugate(Dual));

            return result;
        }

        /// <summary>
        /// Inverts a dual quaternion
        /// </summary>
        /// <returns>The inverse <see cref="DualQuaternion"/>.</returns>
        [Pure]
        public DualQuaternion Inverse()
        {
            var (realLengthSquared, dualLengthSquared) = GetLengthSquared();

            // Real part
            var real = new Quaternion(
                Adjust(-Real.X * realLengthSquared),
                Adjust(-Real.Y * realLengthSquared),
                Adjust(-Real.Z * realLengthSquared),
                Adjust(Real.W * realLengthSquared));

            var a = dualLengthSquared - realLengthSquared;

            // Dual part
            var dual = new Quaternion(
                Adjust(Dual.X * a),
                Adjust(Dual.Y * a),
                Adjust(Dual.Y * a),
                Adjust(Dual.Y * -a));

            // Result
            var result = new DualQuaternion(real, dual);

            return result;
        }

        /// <summary>
        /// Gets the length squared for a dual quaternion.
        /// </summary>
        [Pure]
        public (double realLengthSquared, double dualLengthSquared) GetLengthSquared()
        {
            var real = Real;
            var dual = Dual;

            return (real.LengthSquared(), dual.LengthSquared());
        }

        /// <summary>
        /// Converts a dual quaternion to a 4x4 homogeneous matrix.
        /// </summary>
        /// <returns>The homogeneous <see cref="Matrix4x4"/>.</returns>
        [Pure]
        public Matrix4x4 ToMatrix()
        {
            var q = Normalize(this);

            var w = q.Real.W;
            var x = q.Real.X;
            var y = q.Real.Y;
            var z = q.Real.Z;

            // Extract rotational information
            var result = Matrix4x4.Identity;

            result.V00 = w * w + x * x - y * y - z * z;
            result.V01 = 2 * x * y + 2 * w * z;
            result.V02 = 2 * x * z - 2 * w * y;

            result.V10 = 2 * x * y - 2 * w * z;
            result.V11 = w * w + y * y - x * x - z * z;
            result.V12 = 2 * y * z + 2 * w * x;

            result.V20 = 2 * x * z + 2 * w * y;
            result.V21 = 2 * y * z - 2 * w * x;
            result.V22 = w * w + z * z - x * x - y * y;

            // Extract translation information
            var t = q.Dual * 2.0f * Quaternion.Conjugate(q.Real);

            result.V30 = t.X;
            result.V31 = t.Y;
            result.V32 = t.Z;

            return result;
        }

        /// <summary>
        /// Retrieves a point from a dual quaternion.
        /// </summary>
        /// <param name="round">If true, rounding is applied (Defaulted to false).</param>
        /// <returns>The <see cref="Vector3"/> point.</returns>
        [Pure]
        public Vector3 GetPoint(bool round = false)
        {
            var result = new Vector3(Dual.X, Dual.Y, Dual.Z);
            if (!round) return result;

            // Round
            result.X = Round(result.X);
            result.Y = Round(result.Y);
            result.Z = Round(result.Z);

            return result;
        }

        /// <summary>
        /// Creates an array containing the elements of the dual quaternion.
        /// </summary>
        /// <returns>
        /// An eight-element array containing the components of the dual quaternion.
        /// </returns>
        [Pure]
        public float[] ToArray()
        {
            return new[]
            {
              Real.X, Real.Y, Real.Z, Real.W,
              Dual.X, Dual.Y, Dual.Z, Dual.W
            };
        }

        /// <inheritdoc />
        /// <summary>
        /// Determines whether the specified <see cref="DualQuaternion"/> is equal to this instance.
        /// </summary>
        /// <param name="other">The <see cref="DualQuaternion"/> to compare with this instance.</param>
        /// <returns><c>true</c> if the specified <see cref="DualQuaternion"/> is equal to this instance; otherwise, <c>false</c>.</returns>
        public bool Equals(DualQuaternion other)
        {
            var result =
                Real.Equals(other.Real) &&
                Dual.Equals(other.Dual);

            return result;
        }

        /// <summary>
        /// Determines whether the specified <see cref="object"/> is equal to this instance.
        /// </summary>
        /// <param name="value">The <see cref="T:System.Object"/> to compare with this instance.</param>
        /// <returns><c>true</c> if the specified <see cref="object"/> is equal to this instance; otherwise, <c>false</c>.</returns>
        public override bool Equals(object value)
        {
            if (!(value is DualQuaternion)) return false;
            var other = (DualQuaternion)value;

            return Equals(other);
        }

        /// <summary>
        /// Returns a <see cref="string"/> that represents this instance.
        /// </summary>
        /// <returns>A <see cref="string"/> that represents this instance.</returns>
        public override string ToString()
        {
            return $"({Real}) + ε({Dual})";
        }

        /// <summary>
        /// Returns a hash code for this instance.
        /// </summary>
        /// <returns>A hash code for this instance, suitable for use in hashing algorithms and data structures like a hash table.</returns>
        public override int GetHashCode()
        {
            return (Real.GetHashCode() * 397) ^ Dual.GetHashCode();
        }

        /// <summary>
        /// Adjusts a dual quaternion by setting its component to zero if less than precision.
        /// </summary>
        [Pure]
        public DualQuaternion Adjusted()
        {
            var result = new DualQuaternion(
                Adjust(Real.X), Adjust(Real.Y), Adjust(Real.Z), Adjust(Real.W),
                Adjust(Dual.X), Adjust(Dual.Y), Adjust(Dual.Z), Adjust(Dual.W));

            return result;
        }

        #endregion

        #region Private Methods

        /// <summary>
        /// Adjusts the given double by setting to zero if less than zero tolerance.
        /// </summary>
        /// <param name="value">The double value to adjust.</param>
        /// <returns>The adjusted float value.</returns>
        private static float Adjust(double value)
        {
            var result = value;
            if (Math.Abs(result) < ZeroTolerance) result = 0;

            return (float)result;
        }

        /// <summary>
        /// Rounds a value.
        /// </summary>
        /// <param name="value">The value.</param>
        /// <returns>The <see cref="float"/>.</returns>
        private static float Round(double value)
        {
            var result = Math.Round(value, RoundingDigits);

            return (float)result;
        }

        /// <summary>
        /// Multiplies a quaternion by another.
        /// </summary>
        /// <param name="px">The x component of the left quaternion.</param>
        /// <param name="py">The y component of the left quaternion.</param>
        /// <param name="pz">The z component of the left quaternion.</param>
        /// <param name="pw">The w component of the left quaternion.</param>
        /// <param name="qx">The x component of the right quaternion.</param>
        /// <param name="qy">The y component of the right quaternion.</param>
        /// <param name="qz">The z component of the right quaternion.</param>
        /// <param name="qw">The w component of the right quaternion.</param>
        /// <param name="x">The x component of the product.</param>
        /// <param name="y">The y component of the product.</param>
        /// <param name="z">The z component of the product.</param>
        /// <param name="w">The w component of the product.</param>
        private static void Multiply(
            ref double px,
            ref double py,
            ref double pz,
            ref double pw,
            ref double qx,
            ref double qy,
            ref double qz,
            ref double qw,
            out double x,
            out double y,
            out double z,
            out double w)
        {
            var a = py * qz - pz * qy;
            var b = pz * qx - px * qz;
            var c = px * qy - py * qx;
            var d = px * qx + py * qy + pz * qz;

            x = px * qw + qx * pw + a;
            y = py * qw + qy * pw + b;
            z = pz * qw + qz * pw + c;
            w = pw * qw - d;
        }

        /// <summary>
        /// Combines a dual quaternion transformation to another.
        /// </summary>
        /// <param name="transformation">
        /// The value.
        /// </param>
        /// <param name="x">The x.</param>
        /// <param name="y">The y.</param>
        /// <param name="z">The z.</param>
        /// <param name="w">The w.</param>
        /// <param name="x0">The x 0.</param>
        /// <param name="y0">The y 0.</param>
        /// <param name="z0">The z 0.</param>
        /// <param name="w0">The w 0.</param>
        private static void Combine(
            DualQuaternion transformation,
            ref double x,
            ref double y,
            ref double z,
            ref double w,
            ref double x0,
            ref double y0,
            ref double z0,
            ref double w0)
        {
            // ReSharper disable InconsistentNaming

            /*  We can decompose the problem into quaternion multiplication:            
                Q = q + εq0
                P = p + εp0
                Q*P = q*p + ε(q*p0 + q0*p) 
            */

            var qx = (double)transformation.Real.X;
            var qy = (double)transformation.Real.Y;
            var qz = (double)transformation.Real.Z;
            var qw = (double)transformation.Real.W;

            var q0x = (double)transformation.Dual.X;
            var q0y = (double)transformation.Dual.Y;
            var q0z = (double)transformation.Dual.Z;
            var q0w = (double)transformation.Dual.W;

            var px = x;
            var py = y;
            var pz = z;
            var pw = w;

            var p0x = x0;
            var p0y = y0;
            var p0z = z0;
            var p0w = w0;

            // q*p
            Multiply(ref px, ref qy, ref qz, ref qw, ref px, ref py, ref pz, ref pw, out x, out y, out z, out w);
            Multiply(ref qx, ref qy, ref qz, ref qw, ref p0x, ref p0y, ref p0z, ref p0w, out var ax0, out var ay0, out var az0, out var aw0);
            Multiply(ref q0x, ref q0y, ref q0z, ref q0w, ref px, ref py, ref pz, ref pw, out var bx0, out var by0, out var bz0, out var bw0);

            // a + b
            x0 = ax0 + bx0;
            y0 = ay0 + by0;
            z0 = az0 + bz0;
            w0 = aw0 + bw0;
        }

        #endregion
    }
}