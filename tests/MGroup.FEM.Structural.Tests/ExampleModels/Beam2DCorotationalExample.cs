using MGroup.MSolve.Discretization.Entities;
using MGroup.FEM.Structural.Line;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.Constitutive.Structural.Line;

namespace MGroup.FEM.Structural.Tests.ExampleModels
{
	public class Beam2DCorotationalExample
	{
		public static readonly double expected_solution_node3_TranslationY = 146.5587362562;

		public static Model CreateModel()
		{
			var model = new Model();

			model.SubdomainsDictionary.Add(key: 0, new Subdomain(id: 0));

			var nodes = new[]
			{
				new Node(id: 1, x: 0d, y: 0d),
				new Node(id: 2, x: 100d, y: 0d),
				new Node(id: 3, x: 200d, y: 0d)
			};

			foreach (var node in nodes)
			{
				model.NodesDictionary.Add(node.ID, node);
			}

			var nElems = nodes.Length - 1;
			for (var i = 0; i < nElems; i++)
			{
				var element = new Beam2DCorotational(
					new[]
					{
						model.NodesDictionary[i + 1],
						model.NodesDictionary[i + 2]
					},
					youngModulus: 21000d, poissonRatio: 0.3, density: 7.85, new BeamSection2D(area: 91.04, inertia: 8091d)
				)
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
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.RotationZ, amount: 0d)
				},
				new[]
				{
					new NodalLoad(model.NodesDictionary[nodes.Length], StructuralDof.TranslationY, amount: 20000d)
				}
			));

			return model;
		}
	}
}
