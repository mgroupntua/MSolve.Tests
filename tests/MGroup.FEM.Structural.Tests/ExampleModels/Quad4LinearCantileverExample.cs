using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Discretization;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.FEM.Structural.Continuum;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.Planar;

namespace MGroup.FEM.Structural.Tests.ExampleModels
{
	public class Quad4LinearCantileverExample
	{
		public static readonly double expected_solution_node3_TranslationX = 253.132375961535;

		public static Model CreateModel()
		{
			var model = new Model();

			model.SubdomainsDictionary.Add(key: 0, new Subdomain(id: 0));

			var nodes = new Node[]
			{
				new Node( id: 1, x:  0.0, y:   0.0, z: 0.0 ),
				new Node( id: 2, x: 10.0, y:   0.0, z: 0.0 ),
				new Node( id: 3, x: 10.0, y:  10.0, z: 0.0 ),
				new Node( id: 4, x:  0.0, y:  10.0, z: 0.0 )
			};

			foreach (var node in nodes)
			{
				model.NodesDictionary.Add(node.ID, node);
			}

			var elementFactory = new ContinuumElement2DFactory(
				commonThickness: 1d,
				new ElasticMaterial2D(youngModulus: 3.76, poissonRatio: 0.3779, StressState2D.PlaneStress),
				commonDynamicProperties: null
			);

			var element = elementFactory.CreateElement(CellType.Quad4, nodes);
			element.ID = 0;

			model.ElementsDictionary.Add(element.ID, element);
			model.SubdomainsDictionary[0].Elements.Add(element);

			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(
				new[]
				{
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationX, amount: 0.0 ),
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationY, amount: 0.0 ),
					new NodalDisplacement(model.NodesDictionary[4], StructuralDof.TranslationX, amount: 0.0 ),
					new NodalDisplacement(model.NodesDictionary[4], StructuralDof.TranslationY, amount: 0.0 )
				},
				new[]
				{
					new NodalLoad(model.NodesDictionary[2],StructuralDof.TranslationX, amount: 500d),
					new NodalLoad(model.NodesDictionary[3],StructuralDof.TranslationX, amount: 500d)
				}
			));

			return model;
		}
	}
}
