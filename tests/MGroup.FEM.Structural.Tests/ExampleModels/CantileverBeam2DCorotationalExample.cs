using System.Collections.Generic;
using MGroup.MSolve.Discretization.Entities;
using MGroup.FEM.Structural.Line;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.Constitutive.Structural.Line;

namespace MGroup.FEM.Structural.Tests.ExampleModels
{
	public class CantileverBeam2DCorotationalExample
	{
		public static readonly double expected_solution_node3_TranslationX = -72.090605787610343;

		public static Model CreateModel()
		{
			var model = new Model();

			model.SubdomainsDictionary.Add(key: 0, new Subdomain(id: 0));

			var nodes = new[]
			{
				new Node(id: 1, x: 0.0, y: 0.0),
				new Node(id: 2, x: 100.0, y: 0.0),
				new Node(id: 3, x: 200.0, y: 0.0)
			};

			foreach (var node in nodes)
			{
				model.NodesDictionary.Add(node.ID, node);
			}

			var nElems = nodes.Length - 1;
			var iNode = 1;
			for (var iElem = 0; iElem < nElems; iElem++)
			{
				var element = new Beam2DCorotational(
					new List<INode>(){
						model.NodesDictionary[iNode],
						model.NodesDictionary[iNode + 1]
					},
					youngModulus: 21000d,
					poissonRatio: 0.3,
					density: 7.85,
					new BeamSection2D(area: 91.04, inertia: 8091d))
				{
					ID = iElem + 1
				};

				model.ElementsDictionary.Add(element.ID, element);
				model.SubdomainsDictionary[0].Elements.Add(element);
				iNode++;
			}

			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(
				new[]
				{
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationX, amount: 0d),
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationY, amount: 0d),
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.RotationZ, amount: 0d),
					new NodalDisplacement(model.NodesDictionary[3], StructuralDof.TranslationY, amount: 146d),
				},
				new NodalLoad[] { }
			));

			return model;
		}
	}
}
