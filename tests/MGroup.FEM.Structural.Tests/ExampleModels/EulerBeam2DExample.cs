using System.Collections.Generic;
using MGroup.MSolve.Discretization.Entities;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.Constitutive.Structural;
using MGroup.FEM.Structural.Line;
namespace MGroup.FEM.Structural.Tests.ExampleModels
{
	public class EulerBeam2DExample
	{
		public static readonly double expected_solution4 = 31.388982074929341;
		public static Model CreateModel()
		{
			var model = new Model();

			model.SubdomainsDictionary.Add(key: 0, new Subdomain(id: 0));

			var nodes = new[]
			{
				new Node(id: 1, x: 0d, y: 0d, z: 0d),
				new Node(id: 2, x: 100d, y: 0d, z: 0d),
				new Node(id: 3, x: 200d, y: 0d, z: 0d)
			};

			foreach (var node in nodes)
			{
				model.NodesDictionary.Add(node.ID, node);
			}

			var nElems = nodes.Length - 1;
			for (var i = 0; i < nElems; i++)
			{
				var element = new EulerBeam2D(new List<INode>() { model.NodesDictionary[i + 1], model.NodesDictionary[i + 2] }, youngModulus: 21000d)
				{
					ID = i + 1,
					Density = 7.85,
					SectionArea = 91.04,
					MomentOfInertia = 8091.00,
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
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.RotationZ, amount: 0d)
				},
				new[]
				{
					new NodalLoad(model.NodesDictionary[3], StructuralDof.TranslationY, amount: 2000d)
				}
			));

			return model;
		}
	}
}
