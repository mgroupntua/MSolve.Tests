//using System.Collections.Generic;
//using MGroup.MSolve.Discretization.Entities;
//using MGroup.Constitutive.Structural;
//using MGroup.NumericalAnalyzers.Logging;
//using MGroup.FEM.Structural.Line;
//using MGroup.Constitutive.Structural.BoundaryConditions;
//using MGroup.Solvers.Direct;
//using MGroup.NumericalAnalyzers;
//using MGroup.MSolve.Discretization.Dofs;
//using Xunit;

//namespace MGroup.FEM.Structural.Tests.Integration
//{
//	public class EulerBeam3DTest // TODO: Problem in EulerBeam3D?
//	{
//		private static List<(INode node, IDofType dof)> watchDofs = new List<(INode node, IDofType dof)>();
//		[Fact]
//		public void AppliedDisplacementTestExample()
//		{
//			var model = new Model();

//			model.SubdomainsDictionary.Add(key: 0, new Subdomain(id: 0));

//			var nodes = new Node[]
//			{
//				new Node(id: 1, x: 0.0, y: 0.0, z: 0.0 ),
//				new Node(id: 2, x: 300.0, y: 0.0, z: 0.0 )
//			};

//			foreach (var node in nodes)
//			{
//				model.NodesDictionary.Add(node.ID, node);
//			}

//			var element = new EulerBeam3D(nodes, youngModulus: 21000d, poissonRatio: 0.3)
//			{
//				ID = 1,
//				SectionArea = 1,
//				MomentOfInertiaY = .1,
//				MomentOfInertiaZ = .1,
//				MomentOfInertiaPolar = .1,
//			};

//			model.ElementsDictionary.Add(element.ID, element);
//			model.SubdomainsDictionary[0].Elements.Add(element);

//			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(
//				new List<INodalDisplacementBoundaryCondition>()
//				{
//					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationX, amount: 0d),
//					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.TranslationY, amount: 0d),
//					new NodalDisplacement(model.NodesDictionary[1], StructuralDof.RotationZ, amount: 0d),
//					new NodalDisplacement(model.NodesDictionary[2], StructuralDof.TranslationY, amount: -4.16666666666667E-07),
//				},
//				new List<INodalLoadBoundaryCondition>() { }
//			));

//			var solverFactory = new SkylineSolver.Factory();
//			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
//			var solver = solverFactory.BuildSolver(algebraicModel);
//			var problem = new ProblemStructural(model, algebraicModel, solver);

//			var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);
//			var staticAnalyzer = new StaticAnalyzer(model, algebraicModel, solver, problem, linearAnalyzer);

//			watchDofs.Add((model.NodesDictionary[2], StructuralDof.TranslationX));
//			watchDofs.Add((model.NodesDictionary[2], StructuralDof.RotationZ));

//			linearAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

//			staticAnalyzer.Initialize();
//			staticAnalyzer.Solve();

//			//var results = linearAnalyzer.Logs[0];
//		}
//	}
//}
