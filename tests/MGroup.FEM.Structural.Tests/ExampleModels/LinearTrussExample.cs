using System.Collections.Generic;
using MGroup.MSolve.Discretization.Entities;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.FEM.Structural.Line;

namespace MGroup.FEM.Structural.Tests.ExampleModels
{
	public class LinearTrussExample
	{
		public static readonly double expected_solution0 = 0.00053333333333333336;
		public static readonly double expected_solution1 = 0.0017294083664636196;

		public static Model CreateModel()
		{
			var model = new Model();

			model.SubdomainsDictionary.Add(0, new Subdomain(0));

			var nodes = new[]
			{
				new Node(id: 1, x: 0, y: 0),
				new Node(id: 2, x: 0, y: 40),
				new Node(id: 3, x: 40, y: 40)
			};

			foreach (var node in nodes)
			{
				model.NodesDictionary.Add(node.ID, node);
			}

			var element1 = new Rod2D(
				new List<INode>() { model.NodesDictionary[1], model.NodesDictionary[3] },
				youngModulus: 10e6
			)
			{
				ID = 1,
				Density = 1,
				SectionArea = 1.5
			};

			var element2 = new Rod2D(
				new List<INode>() { model.NodesDictionary[2], model.NodesDictionary[3] },
				youngModulus: 10e6
			)
			{
				ID = 2,
				Density = 1,
				SectionArea = 1.5
			};

			model.ElementsDictionary.Add(element1.ID, element1);
			model.ElementsDictionary.Add(element2.ID, element2);

			model.SubdomainsDictionary[0].Elements.Add(element1);
			model.SubdomainsDictionary[0].Elements.Add(element2);

			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(
				new[]
				{
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationX, 0d),
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationY, 0d),
					new NodalDisplacement(model.NodesDictionary[2], StructuralDof.TranslationX, 0d),
					new NodalDisplacement(model.NodesDictionary[2], StructuralDof.TranslationY, 0d)
				},
				new[]
				{
					new NodalLoad(model.NodesDictionary[3], StructuralDof.TranslationX, amount: 500d),
					new NodalLoad(model.NodesDictionary[3], StructuralDof.TranslationY, amount: 300d)
				}
			));
			return model;
		}
	}
}
