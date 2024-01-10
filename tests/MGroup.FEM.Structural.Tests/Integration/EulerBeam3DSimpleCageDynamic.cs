using System.Collections.Generic;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.Constitutive.Structural;
using MGroup.NumericalAnalyzers;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.NumericalAnalyzers.Discretization.NonLinear;
using MGroup.Solvers.Direct;
using MGroup.FEM.Structural.Tests.ExampleModels;
using Xunit;
using MGroup.NumericalAnalyzers.Dynamic;
using MGroup.MSolve.Discretization.BoundaryConditions;
using System.Reflection.Emit;
using System.IO;
using System.Linq;
using System.Xml.Linq;
using System.Text;
using System;

namespace MGroup.FEM.Structural.Tests.Integration
{
	public class EulerBeam3DSimpleCageDynamic
	{
		private static List<(INode node, IDofType dof)> watchDofs = new List<(INode node, IDofType dof)>();

		[Fact]
		public void SimpleCage()
		{
			var model = EulerBeam3DSimpleCage.CreateModel();
			var log = SolveSimpleModel(model);
			Assert.True(true);
		}

        [Fact]
        public void RaceCage()
        {
            var model = EulerBeam3DRaceCarCage.CreateModel();
            var log = SolveRaceModel(model);
            Assert.True(true);
        }

        private static DOFSLog SolveRaceModel(Model model)
		{
			var headerData = File.ReadAllLines(@"D:\RacecarData\testRace.vtk");
			var solverFactory = new SkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(algebraicModel);
			var problem = new ProblemStructural(model, algebraicModel);

            var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);
            var dynamicAnalyzerBuilder = new NewmarkDynamicAnalyzer.Builder(algebraicModel, problem, linearAnalyzer,
                timeStep: 0.001, totalTime: 60.00, calculateInitialDerivativeVectors: true);
            dynamicAnalyzerBuilder.SetNewmarkParametersForConstantAcceleration();
            var parentAnalyzer = dynamicAnalyzerBuilder.Build();

            parentAnalyzer.ResultStorage = new ImplicitIntegrationAnalyzerLog();
			var nodes = model.EnumerateNodes().ToArray();
            var watchDofs = nodes.Where(x => x.ID % 10 == 0).SelectMany(x => new (INode node, IDofType dof)[]
			{
				(x, StructuralDof.TranslationX),
				(x, StructuralDof.TranslationY),
				(x, StructuralDof.TranslationZ),
			}).ToList();
            linearAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

            parentAnalyzer.Initialize();
            parentAnalyzer.Solve();

            for (int i = 0; i < parentAnalyzer.ResultStorage.Logs.Count / 40; i++)
            {
                var l = (DOFSLog)parentAnalyzer.ResultStorage.Logs[i * 40];
                var logLines = new[]
                {
                    "POINT_DATA 30",
                    "VECTORS displacements double",
                    "0 0 0",
                    $"{l.DOFValues[model.NodesDictionary[10], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[10], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[10], StructuralDof.TranslationZ]}",
                    "0 0 0",
                    $"{l.DOFValues[model.NodesDictionary[20], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[20], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[20], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[30], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[30], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[30], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[40], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[40], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[40], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[50], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[50], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[50], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[60], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[60], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[60], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[70], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[70], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[70], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[80], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[80], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[80], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[90], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[90], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[90], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[100], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[100], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[100], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[110], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[110], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[110], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[120], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[120], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[120], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[130], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[130], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[130], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[140], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[140], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[140], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[150], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[150], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[150], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[160], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[160], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[160], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[170], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[170], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[170], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[180], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[180], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[180], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[190], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[190], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[190], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[200], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[200], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[200], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[210], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[210], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[210], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[220], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[220], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[220], StructuralDof.TranslationZ]}",
                    "0 0 0",
                    $"{l.DOFValues[model.NodesDictionary[230], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[230], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[230], StructuralDof.TranslationZ]}",
                    "0 0 0",
                    $"{l.DOFValues[model.NodesDictionary[240], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[240], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[240], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[250], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[250], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[250], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[260], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[260], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[260], StructuralDof.TranslationZ]}",
                };
                File.WriteAllLines($@"D:\RacecarData\testRace_{i}.vtk", headerData.Concat(logLines));
            }

            var lines = new List<string>
            {
                nodes.Where(x => x.ID % 10 == 0).SelectMany(x => new[]
                {
                    $"{x.ID}-X",
                    $"{x.ID}-Y",
                    $"{x.ID}-Z",
                }).Aggregate(String.Empty, (a, v) => a + v.ToString() + ",")
            };
            lines.AddRange(parentAnalyzer.ResultStorage.Logs
				.Select(x => (DOFSLog)x)
				.Select(v => nodes.Where(x => x.ID % 10 == 0).SelectMany(x => new[]
				{
					v.DOFValues[x, StructuralDof.TranslationX],
					v.DOFValues[x, StructuralDof.TranslationY],
					v.DOFValues[x, StructuralDof.TranslationZ],
				}).Aggregate(String.Empty, (a, v) => a + v.ToString() + ",")));
			File.WriteAllLines(@"D:\RacecarData\RaceResults.csv", lines);

            return null;
		}

        private static DOFSLog SolveSimpleModel(Model model)
        {
            var headerData = File.ReadAllLines(@"D:\RacecarData\testChassis.vtk");
            var solverFactory = new SkylineSolver.Factory();
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemStructural(model, algebraicModel);

            var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);
            var dynamicAnalyzerBuilder = new NewmarkDynamicAnalyzer.Builder(algebraicModel, problem, linearAnalyzer,
                timeStep: 0.001, totalTime: 60.00, calculateInitialDerivativeVectors: true);
            dynamicAnalyzerBuilder.SetNewmarkParametersForConstantAcceleration();
            var parentAnalyzer = dynamicAnalyzerBuilder.Build();

            parentAnalyzer.ResultStorage = new ImplicitIntegrationAnalyzerLog();
            var nodes = model.EnumerateNodes().ToArray();
            var watchDofs = nodes.Where(x => x.ID % 10 == 0).SelectMany(x => new (INode node, IDofType dof)[]
            {
                (x, StructuralDof.TranslationX),
                (x, StructuralDof.TranslationY),
                (x, StructuralDof.TranslationZ),
            }).ToList();
            linearAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

            parentAnalyzer.Initialize();
            parentAnalyzer.Solve();

            for (int i = 0; i < parentAnalyzer.ResultStorage.Logs.Count / 40; i++)
            {
                var l = (DOFSLog)parentAnalyzer.ResultStorage.Logs[i * 40];
                var logLines = new[]
                {
                    "POINT_DATA 13",
                    "VECTORS displacements double",
                    "0 0 0",
                    $"{l.DOFValues[model.NodesDictionary[10], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[10], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[10], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[20], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[20], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[20], StructuralDof.TranslationZ]}",
                    "0 0 0",
                    $"{l.DOFValues[model.NodesDictionary[30], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[30], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[30], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[40], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[40], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[40], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[50], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[50], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[50], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[60], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[60], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[60], StructuralDof.TranslationZ]}",
                    "0 0 0",
                    $"{l.DOFValues[model.NodesDictionary[70], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[70], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[70], StructuralDof.TranslationZ]}",
                    $"{l.DOFValues[model.NodesDictionary[80], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[80], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[80], StructuralDof.TranslationZ]}",
                    "0 0 0",
                    $"{l.DOFValues[model.NodesDictionary[90], StructuralDof.TranslationX]} {l.DOFValues[model.NodesDictionary[90], StructuralDof.TranslationY]} {l.DOFValues[model.NodesDictionary[90], StructuralDof.TranslationZ]}",
                };
                File.WriteAllLines($@"D:\RacecarData\testChassis_{i}.vtk", headerData.Concat(logLines));
            }

            var lines = new List<string>
            {
                nodes.Where(x => x.ID % 10 == 0).SelectMany(x => new[]
                {
                    $"{x.ID}-X",
                    $"{x.ID}-Y",
                    $"{x.ID}-Z",
                }).Aggregate(String.Empty, (a, v) => a + v.ToString() + ",")
            };
            lines.AddRange(parentAnalyzer.ResultStorage.Logs
                .Select(x => (DOFSLog)x)
                .Select(v => nodes.Where(x => x.ID % 10 == 0).SelectMany(x => new[]
                {
                    v.DOFValues[x, StructuralDof.TranslationX],
                    v.DOFValues[x, StructuralDof.TranslationY],
                    v.DOFValues[x, StructuralDof.TranslationZ],
                }).Aggregate(String.Empty, (a, v) => a + v.ToString() + ",")));
            File.WriteAllLines(@"D:\RacecarData\Results.csv", lines);

            return null;
        }
    }
}
