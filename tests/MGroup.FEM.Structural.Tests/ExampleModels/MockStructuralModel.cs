using System.Collections.Generic;
using MGroup.MSolve.Discretization.Entities;
using MGroup.LinearAlgebra.Matrices;
using MGroup.MSolve.Discretization.BoundaryConditions;
using Moq;
using MGroup.Constitutive.Structural;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.Constitutive.Structural.BoundaryConditions;

namespace MGroup.FEM.Structural.Tests.ExampleModels
{
	public class MockStructuralModel
	{
		public static readonly double expected_solution_node0_TranslationX = 2.2840249264795207;
		public static readonly double expected_solution_node0_TranslationY = 2.4351921891904156;

		public static readonly double expected_solution_node0_TranslationX_Explicit = 2.04;
		public static readonly double expected_solution_node0_TranslationY_Explicit = 2.54;

		public static Model CreateModel()
		{
			var model = new Model();

			model.SubdomainsDictionary.Add(key: 0, new Subdomain(id: 0));

			var n = new Node(id: 0, x: double.NaN);
			n.Subdomains.Add(0);
			model.NodesDictionary.Add(0, n);

			var e = new Mock<IStructuralElementType>() { CallBase = true };
			e.Setup(x => x.Nodes).Returns(new[] { n });
			e.Setup(x => x.SubdomainID).Returns(0);
			e.Setup(x => x.StiffnessMatrix()).Returns(Matrix.CreateFromArray(new double[,] { { 6, -2, }, { -2, 4 } }));
			e.Setup(x => x.MassMatrix()).Returns(Matrix.CreateFromArray(new double[,] { { 2, 0, }, { 0, 1 } }));
			e.Setup(x => x.DampingMatrix()).Returns(Matrix.CreateFromArray(new double[,] { { 0, 0, }, { 0, 0 } }));
			e.Setup(x => x.GetElementDofTypes()).Returns(new[] { new[] { StructuralDof.TranslationX, StructuralDof.TranslationY } });
			e.Setup(x => x.MapElementVectorToNodalValues(It.IsAny<double[]>())).Returns((double[] v) => new Dictionary<(INode Node, IDofType DOF), double>
			{
				{ (n, StructuralDof.TranslationX), v[0] },
				{ (n, StructuralDof.TranslationY), v[1] },
			});
			e.Setup(x => x.MapNodalBoundaryConditionsToElementVector(It.IsAny<IEnumerable<INodalBoundaryCondition<IDofType>>>(), It.IsAny<double[]>()))
				.Returns((IEnumerable<INodalBoundaryCondition<IDofType>> bcs, double[] v) =>
				{
					foreach (var bc in bcs)
					{
						if (bc.DOF == StructuralDof.TranslationX)
						{
							v[0] = bc.Amount;
						}
						if (bc.DOF == StructuralDof.TranslationY)
						{
							v[1] = bc.Amount;
						}
					}

					return true;
				});
			e.SetupGet(x => x.DofEnumerator).Returns(new GenericDofEnumerator());
			model.ElementsDictionary.Add(0, e.Object);
			model.SubdomainsDictionary[0].Elements.Add(e.Object);

			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(null, new[] { new NodalLoad(n, StructuralDof.TranslationY, 10d) }));
			model.BoundaryConditions.Add(new StructuralTransientBoundaryConditionSet
			(
				new[] { new StructuralBoundaryConditionSet(
					new[] { new NodalAcceleration(n, StructuralDof.TranslationY, 10d) },
					null
				)},
				(t, amount) => t == 0d ? amount : 0d
			));

			return model;
		}
	}
}
