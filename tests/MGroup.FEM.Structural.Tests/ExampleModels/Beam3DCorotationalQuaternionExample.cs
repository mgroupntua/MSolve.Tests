using System.Collections.Generic;
using MGroup.MSolve.Discretization.Entities;
using MGroup.FEM.Structural.Line;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.Line;
using MGroup.Constitutive.Structural.BoundaryConditions;

namespace MGroup.FEM.Structural.Tests.ExampleModels
{
	public class Beam3DElasticCorotationalQuaternionExample
	{
		public static readonly double expected_solution7 = 148.936792350562;

		public static Model CreateModel()
		{
			var model = new Model();

			model.SubdomainsDictionary.Add(key: 0, new Subdomain(id: 0));

			var nodes = new[]
			{
				new Node(id: 1, x: 0d, y: 0d, z: 0d),
				new Node(id: 2, x: 300d, y: 0d, z: 0d),
				new Node(id: 3, x: 600d, y: 0d, z: 0d)
			};

			foreach (var node in nodes)
			{
				model.NodesDictionary.Add(node.ID, node);
			}

			var nElems = nodes.Length - 1;
			for (var i = 0; i < nElems; i++)
			{
				var elementNodes = new List<INode>()
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
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.RotationZ, amount: 0d)
				},
				new[]
				{
					new NodalLoad(model.NodesDictionary[3], StructuralDof.TranslationY, amount: 20000d)
				}
			));

			return model;
		}
	}
}
