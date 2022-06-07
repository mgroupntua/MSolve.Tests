using System.Linq;
using MGroup.MSolve.Discretization.Entities;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.Constitutive.Structural;
using MGroup.FEM.Structural.Line;
using MGroup.Constitutive.Structural.Line;

namespace MGroup.FEM.Structural.Tests.ExampleModels
{
	public class NLBeamExample
	{
		public static Model CreateModel()
		{
			var model = new Model();

			model.SubdomainsDictionary.Add(key: 0, new Subdomain(id: 0));

			model.NodesDictionary.Add(1, new Node(id: 1, x: 0, y: 0, z: 0));
			model.NodesDictionary.Add(2, new Node(id: 2, x: 5, y: 0, z: 0));

			model.ElementsDictionary.Add(1, new Beam3DCorotationalQuaternion(
				model.NodesDictionary.Values.ToList(),
				youngModulus: 2.1e6,
				poissonRatio: 0.2,
				density: 1,
				new BeamSection3D(area: 0.06, inertiaY: 0.0002, inertiaZ: 0.00045, torsionalInertia: 0.000818, effectiveAreaY: 0.05, effectiveAreaZ: 0.05)
			)
			{
				ID = 1,
			});
			model.SubdomainsDictionary[0].Elements.Add(model.ElementsDictionary[1]);

			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(
				new[]
				{
					new NodalDisplacement(model.NodesDictionary[1],StructuralDof.TranslationX, amount: 0d),
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationY, amount: 0d),
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationZ, amount: 0d),
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.RotationX, amount: 0d),
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.RotationY, amount: 0d),
					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.RotationZ, amount: 0d)
				},
				new[]
				{   new NodalLoad(model.NodesDictionary[2], StructuralDof.TranslationY, amount: 100d)   }
			));

			return model;
		}
	}
}
