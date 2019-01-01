// --------------------------------------------------------------------------------------------------------------------
// <copyright file="DualQuaternionTests.cs" company="Stéphane Pareilleux">
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

using System;
using System.Numerics;

using NUnit.Framework;

namespace DQNet.Tests
{
    /// <summary>
    /// The dual quaternion tests.
    /// </summary>
    [TestFixture]
    public class DualQuaternionTests
    {
        /// <summary>
        /// Simple dual quaternion test 1.
        /// </summary>
        [TestCase]
        public void SimpleDualQuaternionTest()
        {
            // Translation 5 units in X direction
            var translation = DualQuaternion.CreateFromTranslation(5, Vector3.UnitX);

            // Rotation of PI around Z axis centered on origin - We are using Plücker coordinates to describe the axis of rotation
            var rotation = DualQuaternion.CreateRotationPlucker(Math.PI, Vector3.UnitZ, Vector3.Zero);

            // We compose displacements, translation is applied first
            var displacement = rotation * translation;

            // We apply the point transform (Clifford conjugation for points) on the origin point using the displacement
            var point = displacement.TransformPoint();

            // Assert
            Assert.That(point, Is.EqualTo(new Vector3(-5, 0, 0)));
        }

        /// <summary>
        /// Simple dual quaternion test 2.
        /// </summary>
        [TestCase]
        public void SimpleDualQuaternionTest2()
        {
            // Translation 5 units in X direction
            var translation = DualQuaternion.CreateFromTranslation(5, Vector3.UnitX);

            // Rotation of PI/2 around Z axis centered on origin - We are using Plücker coordinates to describe the axis of rotation
            var rotation = DualQuaternion.CreateRotationPlucker(Math.PI / 2f, Vector3.UnitZ, Vector3.Zero);

            // We compose displacements, translation is applied first
            var displacement = rotation * translation;

            // We apply the point transform (Clifford conjugation for points) on the origin point using the displacement
            var point = displacement.TransformPoint();

            // Assert
            Assert.That(point, Is.EqualTo(new Vector3(0, 5, 0)));
        }

        /// <summary>
        /// The EPSON E2L SCARA robot test (Reference configuration).
        /// </summary>
        [TestCase]
        public void EpsonE2LScaraRobotReference()
        {
            // Arrange
            var pose = new ScaraRobotPose(0, 0, 0, 0);

            // Act
            var point = ScaraForwardKinematics(pose);

            // Assert
            Assert.That(point, Is.EqualTo(new Vector3(650, 0, 318)));
        }

        /// <summary>
        /// The EPSON E2L SCARA robot test (Lower to X/Y plane).
        /// </summary>
        [TestCase]
        public void EpsonE2LScaraRobotLowerXyPlane()
        {
            // Arrange
            var pose = new ScaraRobotPose(0, 0, 0, 318);

            // Act
            var point = ScaraForwardKinematics(pose);

            // Assert
            Assert.That(point, Is.EqualTo(new Vector3(650, 0, 0)));
        }

        /// <summary>
        /// The EPSON E2L SCARA robot test (Rotations).
        /// </summary>
        [TestCase]
        public void EpsonE2LScaraRobotTestRotations()
        {
            // Arrange
            var pose = new ScaraRobotPose(-Math.PI / 2, -Math.PI / 2, 0, 318);

            // Act
            var point = ScaraForwardKinematics(pose);

            // Assert
            Assert.That(point, Is.EqualTo(new Vector3(-350, -300, 0)));
        }

        /// <summary>
        /// Compute the coordinates of the end activator for a EPSON SCARA robot.
        /// </summary>
        /// <param name="pose">
        /// The robot pose.
        /// </param>
        /// <returns>
        /// The coordinates of the end effector.
        /// </returns>
        private static Vector3 ScaraForwardKinematics(ScaraRobotPose pose)
        {
            // Here we create the three rotation quaternions
            // Note that we are directly using the Plücker coordinates of the joint axes. 
            // However, rotation would work for axes defined by an orientation and a point belonging to the line.
            var r1 = DualQuaternion.CreateFromRotation(pose.Theta1, Vector3.UnitZ, new Vector3(0, 0, 0));
            var r2 = DualQuaternion.CreateFromRotation(pose.Theta2, Vector3.UnitZ, new Vector3(300, 0, 0));
            var r3 = DualQuaternion.CreateFromRotation(pose.Theta3, Vector3.UnitZ, new Vector3(650, 0, 0));

            // Fourth joint is translation
            var t = DualQuaternion.CreateFromTranslation(pose.Z, -Vector3.UnitZ);

            // The end effector position for the reference configuration
            var endEffector = DualQuaternion.CreateFromTranslation(new Vector3(650, 0, 318));

            // Here we combine the four displacements into a single one and return it. 
            // Notice we also append the end-effector at the reference configuration. 
            // The order of application of the transformations is right-most is applied first.
            var displacements = DualQuaternion.Multiply(r1, r2, r3, t, endEffector);

            // Here we basically use the action f4g (specified in libdq’s manual) to transform the origin point.
            // This will give us the forward kinematics of the robot.
            var result = displacements.TransformPoint();

            return result;
        }

        /// <summary>
        /// Represents a pose for an EPSON E2L SCARA robot.
        /// </summary>
        private class ScaraRobotPose
        {
            public ScaraRobotPose(
                double theta1, 
                double theta2, 
                double theta3, 
                double z)
            {
                Theta1 = theta1;
                Theta2 = theta2;
                Theta3 = theta3;
                Z = z;
            }

            /// <summary>
            /// Gets the Theta1 angle.
            /// </summary>
            public double Theta1 { get; }

            /// <summary>
            /// Gets the Theta2 angle.
            /// </summary>
            public double Theta2 { get; }

            /// <summary>
            /// Gets the Theta3 angle.
            /// </summary>
            public double Theta3 { get; }

            /// <summary>
            /// Gets the Z translation.
            /// </summary>
            public double Z { get; }
        }
    }
}