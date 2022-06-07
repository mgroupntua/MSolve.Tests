using System.Collections.Generic;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Discretization;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.Planar;
using MGroup.FEM.Structural.Continuum;
using MGroup.Constitutive.Structural.BoundaryConditions;

namespace MGroup.FEM.Structural.Tests.ExampleModels
{
	public class QuadCantileverExample1
	{
		public static Model CreateModel()
		{
			var model = new Model();

			model.SubdomainsDictionary.Add(key: 0, new Subdomain(id: 0));

			var indexNode = 0;
			for (var i = 0; i < 25; i++)
			{
				for (var j = 0; j < 5; j++)
				{
					model.NodesDictionary.Add(indexNode, new Node(id: indexNode++, x: i, y: j, z: 0.0));
				}
			}

			var factory = new ContinuumElement2DFactory(
				commonThickness: 1d,
				new ElasticMaterial2D(youngModulus: 3.0e07, poissonRatio: 0.3, StressState2D.PlaneStress),
				commonDynamicProperties: null);

			var indexElement = 0;
			for (var i = 0; i < 24; i++)
			{
				for (var j = 0; j < 4; j++)
				{
					var element = factory.CreateElement(CellType.Quad4, new[]
					{
						model.NodesDictionary[i * 5 + j],
						model.NodesDictionary[(i + 1) * 5 + j],
						model.NodesDictionary[(i + 1) * 5 + j + 1],
						model.NodesDictionary[i * 5 + j + 1]
					});
					element.ID = indexElement;

					model.ElementsDictionary.Add(indexElement, element);
					model.SubdomainsDictionary[0].Elements.Add(element);

					indexElement++;
				}
			}

			var constraints = new List<INodalDisplacementBoundaryCondition>();
			for (var i = 0; i < 5; i++)
			{
				constraints.Add(new NodalDisplacement(model.NodesDictionary[i], StructuralDof.TranslationX, 0d));
				constraints.Add(new NodalDisplacement(model.NodesDictionary[i], StructuralDof.TranslationY, 0d));
			}

			var loads = new[]
			{
				new NodalLoad(model.NodesDictionary[124], StructuralDof.TranslationY, amount: 1000d)
			};

			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(constraints, loads));

			return model;
		}
	}
}
