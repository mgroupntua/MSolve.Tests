using MGroup.MSolve.Discretization.Entities;
using MGroup.FEM.Structural.Line;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.BoundaryConditions;

namespace MGroup.FEM.Structural.Tests.ExampleModels
{
	public class CantileverBeam2DExample
	{
		public static readonly double expected_solution_node2_TranslationY = -2.08333333333333333e-5;
		public static Model CreateModel()
		{
			var model = new Model();

			model.SubdomainsDictionary.Add(key: 0, new Subdomain(id: 0));

			var nodes = new Node[]
			{
				new Node(id: 1, x: 0.0, y: 0.0, z: 0.0),
				new Node(id: 2, x: 5.0, y: 0.0, z: 0.0)
			};

			foreach (var node in nodes)
			{
				model.NodesDictionary.Add(node.ID, node);
			}

			var element = new EulerBeam2D(nodes, youngModulus: 2.0e08)
			{
				ID = 1,
				SectionArea = 1,
				MomentOfInertia = .1
			};

			model.ElementsDictionary.Add(element.ID, element);
			model.SubdomainsDictionary[0].Elements.Add(element);

			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(
				new[]
				{
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationX, 0d),
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationY, 0d),
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.RotationZ, 0d)
				},
				new[]
				{
					new NodalLoad(model.NodesDictionary[2], StructuralDof.TranslationY, amount: -10d)
				}
			));

			return model;
		}
	}
}
