// --------------------------------------------------------------------------------------------------------------------
// <copyright file="DualQuaternionExtensions.cs" company="Stéphane Pareilleux">
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
// --------------------------------------------------------------------------------------------------------------------

using System.Numerics;

namespace DQNet
{
    /// <summary>
    /// Dual quaternion extension methods.
    /// </summary>
    public static class DualQuaternionExtensions
    {
        /// <summary>
        /// Applies a Clifford conjugation transformation of type F4G, then convert the result to a point (Applied to a specified point).
        /// </summary>
        /// <param name="transformation">The <see cref="DualQuaternion"/> representing the transformation.</param>
        /// <param name="value">The <see cref="Vector3"/> point being transformed.</param>
        /// <param name="round">If true, applies rounding to the transformed point coordinates (Defaulted to true).</param>
        /// <returns>The transformed <see cref="Vector3"/> point.</returns>
        public static Vector3 TransformPoint(
            this in DualQuaternion transformation, 
            Vector3 value, 
            bool round = true)
        {
            return DualQuaternion.TransformPoint(value, transformation, round);
        }

        /// <summary>
        /// Applies a Clifford conjugation transformation of type F4G, then convert the result to a point (Applied to the origin point).
        /// </summary>
        /// <param name="transformation">The <see cref="DualQuaternion"/> transformation.</param>
        /// <param name="round">If true, applies rounding to the transformed point coordinates (Defaulted to true).</param>
        /// <returns>The transformed <see cref="Vector3"/> point.</returns>
        public static Vector3 TransformPoint(
            this in DualQuaternion transformation, 
            bool round = true)
        {
            return DualQuaternion.TransformPoint(
                Vector3.Zero, 
                transformation, 
                round);
        }
    }
}