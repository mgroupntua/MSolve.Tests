using System.Collections.Generic;
using MGroup.Constitutive.Structural;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Solution;
using MGroup.NumericalAnalyzers;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.NumericalAnalyzers.Discretization.NonLinear;
using MGroup.Solvers.Direct;
using MGroup.FEM.Structural.Tests.ExampleModels;
using MGroup.FEM.Structural.Tests.Commons;
using Xunit;
using modelBuilder = MGroup.FEM.Structural.Tests.ExampleModels.Hexa8ContinuumNonLinearCantileverDynamicExample;
using System.Linq;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.MSolve.Discretization.BoundaryConditions;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.DofOrdering.Reordering;
using MGroup.NumericalAnalyzers.Dynamic;
using System;

namespace MGroup.FEM.Structural.Tests.Integration
{
	public static class HexaCantileverContinuumDynamic
	{   // ORIGIN: PlateRevisitedLessConstrainted.cs from Preliminary2_Copy repo
		// changes: updated to new msolve and for dynamic problem implementation

		//Edw epilegontai oi parametroi ts analushs
		public static int NR_steps = 1;
        private static double timestep=0.0005; // those are overwritten for the periodic example
        private static double totalTime=0.08; // those are overwritten for the periodic example

		//Oi parametroi tou mondelou kai twn constrains  allazoun sto modelBuilder dld PlateRevisitedLessConstrainted

		[Fact]
		private static void RunSuddenLoadTest()
		{
			modelBuilder.monitoredDof = StructuralDof.TranslationX;
			Model model = modelBuilder.CreateModel();
			modelBuilder.AddStaticNodalLoads(model);

			double[] computedDisplacements = SolveModelDynamic(model);
			Assert.True(Utilities.AreDisplacementsSame(modelBuilder.GetExpectedDisplacementsSuddenLoad(), computedDisplacements, tolerance: 2E-3));
		}

		[Fact]
		private static void RunSuddenLoadWithInitialDisplacementsTest()
		{
			modelBuilder.monitoredDof = StructuralDof.TranslationX;
			Model model = modelBuilder.CreateModel();
			modelBuilder.AddStaticNodalLoads(model);
			modelBuilder.AddInitialConditionsDisplacements(model);
			double[] computedDisplacements = SolveModelDynamic(model);
			//TODO TO PRWTO VMA PREPEI NA PETIETAI KAI H SUGKRISI NA XEKINA ME TO DEFTERO
			Assert.True(Utilities.AreDisplacementsSame(modelBuilder.GetExpectedDisplacementsSuddenLoadAndInitialConditionsDisplacements(), computedDisplacements, tolerance: 1E-4));
		}




		[Fact]
		private static void RunTransientTestNoDelay()
		{
			modelBuilder.monitoredDof = StructuralDof.TranslationX;
			Model model = modelBuilder.CreateModel();
			modelBuilder.AddTransientLoadNoDelay(model);

			double[] computedDisplacements = SolveModelDynamic(model);
			Assert.True(Utilities.AreDisplacementsSame(modelBuilder.GetExpectedDisplacementsTransientLoadNoDelayADINA(), 
																computedDisplacements, tolerance: 1e-5));
		}

		[Fact]
		private static void RunTransientTestWithDelay()
		{
			modelBuilder.monitoredDof = StructuralDof.TranslationX;
			Model model = modelBuilder.CreateModel();
			modelBuilder.AddTransientLoadWithDelay(model);

			double[] computedDisplacements = SolveModelDynamic(model);
			Assert.True(Utilities.AreDisplacementsSame(modelBuilder.GetExpectedDisplacementsTransientLoadWithDelayADINA(),
																computedDisplacements, tolerance: 1e-5));
		}
			[Fact]
		private static void RunTransientTestPeriodic()
		{
			timestep = 0.0005; totalTime = 0.16;
			modelBuilder.monitoredDof = StructuralDof.TranslationX;
			Model model = modelBuilder.CreateModel();
			modelBuilder.AddPeriodicTransientLoad(model);

			double[] computedDisplacements = SolveModelDynamic(model);
			timestep = 0.0005;totalTime = 0.08;
			Assert.True(Utilities.AreDisplacementsSame(modelBuilder.GetExpectedDisplacementsPeriodicLoadADINA(),
																computedDisplacements, tolerance: 1e-5));
		}

		private static double[] SolveModelDynamic(Model model)
		{
			var solverFactory = new SkylineSolver.Factory() { DofOrderer = new DofOrderer(new NodeMajorDofOrderingStrategy(), new NodeMajorReordering()) };
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(algebraicModel);
			var problem = new ProblemStructural(model, algebraicModel);

			var loadControlAnalyzerBuilder = new LoadControlAnalyzer.Builder( algebraicModel, solver, problem, numIncrements: NR_steps)
			{
				ResidualTolerance = 1E-10,
				MaxIterationsPerIncrement = 100,
				NumIterationsForMatrixRebuild = 1
			};
			var loadControlAnalyzer = loadControlAnalyzerBuilder.Build();


			//var staticAnalyzer = new StaticAnalyzer(model, algebraicModel, problem, loadControlAnalyzer);
			var dynamicAnalyzerBuilder = new NewmarkDynamicAnalyzer.Builder( algebraicModel, problem, loadControlAnalyzer,
				timeStep: timestep, totalTime: totalTime);
			dynamicAnalyzerBuilder.SetNewmarkParametersForConstantAcceleration();
			NewmarkDynamicAnalyzer parentAnalyzer = dynamicAnalyzerBuilder.Build();
			parentAnalyzer.ResultStorage = new ImplicitIntegrationAnalyzerLog();



			//logs 
			var node_A = modelBuilder.monitoredNode;

			var emptyBCs = new List<INodalBoundaryCondition>();


			var loggerA = new TotalLoadsDisplacementsPerIncrementLog(model.NodesDictionary[node_A], modelBuilder.monitoredDof, emptyBCs, algebraicModel,
				$"hexaContinuumCantileverDynamicResults.txt");
			

			loadControlAnalyzer.IncrementalLog = loggerA;
			List<(INode node, IDofType dof)> watchDofs = new List<(INode node, IDofType dof)>();
			watchDofs.Add((model.NodesDictionary[node_A], modelBuilder.monitoredDof));
			loadControlAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);
			
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();

			int totalNewmarkstepsNum = (int)Math.Truncate(totalTime / timestep);
			var totalDisplacementOverTime = new double[totalNewmarkstepsNum];
			for (int i1 = 0; i1 < totalNewmarkstepsNum; i1++)
            {
				var timeStepResultsLog = parentAnalyzer.ResultStorage.Logs[i1];
				totalDisplacementOverTime[i1] = ((DOFSLog)timeStepResultsLog).DOFValues[model.GetNode(node_A), modelBuilder.monitoredDof];
			}


			return totalDisplacementOverTime;



		}
	}

}
