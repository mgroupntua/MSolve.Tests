using System.Collections.Generic;
using MGroup.MSolve.Discretization.Entities;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.Constitutive.Structural;
using MGroup.FEM.Structural.Line;
using MGroup.Constitutive.Structural.Line;

namespace MGroup.FEM.Structural.Tests.ExampleModels
{
	public class PlaneFrameExample
	{
		public static readonly double expected_solution_node2_TranslationX = 120.1108698752;
		public static Model CreateModel()
		{
			var model = new Model();

			model.SubdomainsDictionary.Add(key: 0, new Subdomain(id: 0));

			var nodes = new[]
			{
				new Node(id: 1, x: 0.0, y: 0.0, z: 0.0),
				new Node(id: 2, x: 0.0, y: 100.0, z: 0.0),
				new Node(id: 3, x: 100.0, y: 100.0, z: 0.0),
				new Node(id: 4, x: 100.0, y: 0.0, z: 0.0)
			};

			foreach (var node in nodes)
			{
				model.NodesDictionary.Add(node.ID, node);
			}

			var nElems = 3;
			for (var i = 0; i < nElems; i++)
			{
				IReadOnlyList<INode> elementNodes = new List<INode>()
				{
					model.NodesDictionary[i + 1],
					model.NodesDictionary[i + 2]
				};

				var beamSection = new BeamSection3D(area: 91.04, inertiaY: 2843d, inertiaZ: 8091d, torsionalInertia: 76.57, effectiveAreaY: 91.04, effectiveAreaZ: 91.04);
				var element = new Beam3DCorotationalQuaternion(elementNodes, youngModulus: 21000d, poissonRatio: 0.3, density: 7.85, beamSection)
				{
					ID = i + 1
				};

				model.ElementsDictionary.Add(element.ID, element);
				model.SubdomainsDictionary[0].Elements.Add(element);
			}

			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(
				new[]
				{
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationX, amount: 0d),
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationY, amount: 0d),
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationZ, amount: 0d),
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.RotationX, amount: 0d),
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.RotationY, amount: 0d),
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.RotationZ, amount: 0d),
					new NodalDisplacement(model.NodesDictionary[4], StructuralDof.TranslationX, amount: 0d),
					new NodalDisplacement(model.NodesDictionary[4], StructuralDof.TranslationY, amount: 0d),
					new NodalDisplacement(model.NodesDictionary[4], StructuralDof.TranslationZ, amount: 0d),
					new NodalDisplacement(model.NodesDictionary[4], StructuralDof.RotationX, amount: 0d),
					new NodalDisplacement(model.NodesDictionary[4], StructuralDof.RotationY, amount: 0d),
					new NodalDisplacement(model.NodesDictionary[4], StructuralDof.RotationZ, amount: 0d)
				},
				new[]
				{
					new NodalLoad(model.NodesDictionary[2], StructuralDof.TranslationX, amount: 500000d)
				}
			));

			return model;
		}
	}
}
