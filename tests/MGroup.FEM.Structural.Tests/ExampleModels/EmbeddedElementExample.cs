using System.Linq;
using System.Collections.Generic;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Numerics.Integration.Quadratures;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.Continuum;
using MGroup.FEM.Structural.Line;
using MGroup.FEM.Structural.Continuum;
using MGroup.FEM.Structural.Embedding;
using MGroup.Constitutive.Structural.Line;

namespace MGroup.FEM.Structural.Tests.ExampleModels
{
	public class EmbeddedElementExample
	{
		public static readonly double expected_solution_node8_TranslationZ = 11.584726466617692;

		public static Model CreateModel()
		{
			var model = new Model();
			model.SubdomainsDictionary.Add(key: 0, new Subdomain(id: 0));
			HostElementsBuilder(model);
			EmbeddedElementsBuilder(model);

			var embeddedGrouping = new EmbeddedGrouping(
				model,
				model.ElementsDictionary
					.Where(x => x.Key == 1)
					.Select(kv => kv.Value),
				model.ElementsDictionary.
					Where(x => x.Key == 2)
					.Select(kv => kv.Value),
				true);

			return model;
		}

		private static void HostElementsBuilder(Model model)
		{
			var nodes = new[]
			{
				new Node(id: 1, x: 10.00, y: 2.50, z: 2.50),
				new Node(id: 2, x: 0.00, y: 2.50, z: 2.50),
				new Node(id: 3, x: 0.00, y: -2.50, z: 2.50),
				new Node(id: 4, x: 10.00, y: -2.50, z: 2.50),
				new Node(id: 5, x: 10.00, y: 2.50, z: -2.50),
				new Node(id: 6, x: 0.00, y: 2.50, z: -2.50),
				new Node(id: 7, x: 0.00, y: -2.50, z: -2.50),
				new Node(id: 8, x: 10.00, y: -2.50, z: -2.50)
			};

			foreach (var node in nodes)
			{
				model.NodesDictionary.Add(node.ID, node);
			}

			var hexa8NLelement = new Hexa8NonLinear(
				nodes,
				new ElasticMaterial3D(youngModulus: 3.76, poissonRatio: 0.3779),
				GaussLegendre3D.GetQuadratureWithOrder(orderXi: 3, orderEta: 3, orderZeta: 3)
			)
			{
				ID = 1
			};

			model.ElementsDictionary.Add(hexa8NLelement.ID, hexa8NLelement);
			model.SubdomainsDictionary[0].Elements.Add(hexa8NLelement);

			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(
				new[]
				{
					new NodalDisplacement(model.NodesDictionary[2],StructuralDof.TranslationX, 0d),
					new NodalDisplacement(model.NodesDictionary[2],StructuralDof.TranslationY, 0d),
					new NodalDisplacement(model.NodesDictionary[2],StructuralDof.TranslationZ, 0d),
					new NodalDisplacement(model.NodesDictionary[3],StructuralDof.TranslationX, 0d),
					new NodalDisplacement(model.NodesDictionary[3],StructuralDof.TranslationY, 0d),
					new NodalDisplacement(model.NodesDictionary[3],StructuralDof.TranslationZ, 0d),
					new NodalDisplacement(model.NodesDictionary[6],StructuralDof.TranslationX, 0d),
					new NodalDisplacement(model.NodesDictionary[6],StructuralDof.TranslationY, 0d),
					new NodalDisplacement(model.NodesDictionary[6],StructuralDof.TranslationZ, 0d),
					new NodalDisplacement(model.NodesDictionary[7],StructuralDof.TranslationX, 0d),
					new NodalDisplacement(model.NodesDictionary[7],StructuralDof.TranslationY, 0d),
					new NodalDisplacement(model.NodesDictionary[7],StructuralDof.TranslationZ, 0d)
				},
				new[]
				{
					new NodalLoad(model.NodesDictionary[1], StructuralDof.TranslationZ, 25d),
					new NodalLoad(model.NodesDictionary[4], StructuralDof.TranslationZ, 25d),
					new NodalLoad(model.NodesDictionary[5], StructuralDof.TranslationZ, 25d),
					new NodalLoad(model.NodesDictionary[8], StructuralDof.TranslationZ, 25d)
				}
			));
		}

		private static void EmbeddedElementsBuilder(Model model)
		{
			model.NodesDictionary.Add(9, new Node(id: 9, x: 0.00, y: 0.00, z: 0.00));
			model.NodesDictionary.Add(10, new Node(id: 10, x: 10.00, y: 0.00, z: 0.00));

			var beamSection = new BeamSection3D(area: 1776.65, inertiaY: 1058.55, inertiaZ: 1058.55, torsionalInertia: 496.38, effectiveAreaY: 1776.65, effectiveAreaZ: 1776.65);
			IReadOnlyList<INode> elementNodes = new List<INode>()
			{
				model.NodesDictionary[9],
				model.NodesDictionary[10]
			};
			var beamElement = new Beam3DCorotationalQuaternion(elementNodes, youngModulus: 1d, poissonRatio: (1d / (2 * 1d)) - 1d, density: 7.85, beamSection)
			{
				ID = 2
			};

			model.ElementsDictionary.Add(beamElement.ID, beamElement);
			model.SubdomainsDictionary[0].Elements.Add(beamElement);
		}
	}
}
