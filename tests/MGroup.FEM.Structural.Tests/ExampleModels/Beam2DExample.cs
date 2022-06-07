using System.Collections.Generic;
using MGroup.MSolve.Discretization.Entities;
using MGroup.Constitutive.Structural;
using MGroup.FEM.Structural.Line;
using MGroup.Constitutive.Structural.BoundaryConditions;

namespace MGroup.FEM.Structural.Tests.ExampleModels
{
	public class Beam2DExample
	{
		public static readonly double expected_solution1 = 2.2840249264795207;
		public static Model CreateModel()
		{
			var model = new Model();

			model.SubdomainsDictionary.Add(key: 1, new Subdomain(id: 1));

			var nodes = new[]
			{
				new Node(id: 1, x: 0d, y: 0d, z: 0d),
				new Node(id: 2, x: 300d, y: 0d, z: 0d)
			};

			foreach (var node in nodes)
			{
				model.NodesDictionary.Add(node.ID, node);
			}

			var element = new EulerBeam2D(new List<INode>() { model.NodesDictionary[1], model.NodesDictionary[2] }, youngModulus: 21000d)
			{
				ID = 1,
				Density = 7.85,
				SectionArea = 91.04,
				MomentOfInertia = 8091d
			};

			model.ElementsDictionary.Add(element.ID, element);
			model.SubdomainsDictionary[1].Elements.Add(element);

			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(
				new List<INodalDisplacementBoundaryCondition>()
				{
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationX, amount: 0d),
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationY, amount: 0d),
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.RotationZ, amount: 0d)
				},
				new List<INodalLoadBoundaryCondition>()
				{
					new NodalLoad(model.NodesDictionary[nodes.Length], StructuralDof.TranslationY, amount: 1000d)
				}
			));

			return model;
		}
	}
}
