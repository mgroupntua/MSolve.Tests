using MGroup.Constitutive.Structural;
using MGroup.FEM.Structural.Tests.Commons;
using MGroup.FEM.Structural.Tests.ExampleModels;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Entities;
using MGroup.NumericalAnalyzers;
using MGroup.NumericalAnalyzers.Discretization.NonLinear;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.Solvers.Direct;
using System.Collections.Generic;
using System.IO;
using Xunit;

namespace MGroup.FEM.Structural.Tests.Integration
{
    public static class Hexa8ViscoelasticNonLinearTest
    {
        [Fact]
        private static void RunTest()
        {
            //var model = Hexa8OneElementExampleViscoPlastic.CreateModel();
            var model = Hexa8OneElementExampleViscoElastic.CreateModel();
            var computedDisplacements = SolveModel(model);
            //WriteToFile("C:\\Users\\DELL\\Source\\Repos\\dsavvas\\MSolve.Tests\\results.txt", false, computedDisplacements);
            //Assert.True(Utilities.AreDisplacementsSamePerIncrement(Hexa8OneElementExampleViscoElastic.GetExpectedDisplacements(), computedDisplacements, tolerance: 1e-5));
            Assert.True(Utilities.AreDisplacementsSame(Hexa8OneElementExampleViscoElastic.GetExpectedDisplacements(), computedDisplacements, tolerance: 1E-7));
        }

        private static IncrementalDisplacementsLog SolveModel(Model model)
        {
            var solverFactory = new SkylineSolver.Factory();
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemStructural(model, algebraicModel, solver);

            var loadControlAnalyzerBuilder = new LoadControlAnalyzer.Builder(model, algebraicModel, solver, problem, numIncrements: 5)
            {
                ResidualTolerance = 1E-8,
                MaxIterationsPerIncrement = 100,
                NumIterationsForMatrixRebuild = 1
            };
            var loadControlAnalyzer = loadControlAnalyzerBuilder.Build();
            var staticAnalyzer = new StaticAnalyzer(model, algebraicModel, solver, problem, loadControlAnalyzer);

            loadControlAnalyzer.IncrementalDisplacementsLog = new IncrementalDisplacementsLog(
                new List<(INode node, IDofType dof)>()
                {
                    (model.NodesDictionary[1], StructuralDof.TranslationX),
                    (model.NodesDictionary[4], StructuralDof.TranslationX),
                    (model.NodesDictionary[8], StructuralDof.TranslationX),
                    (model.NodesDictionary[5], StructuralDof.TranslationX)
                }, algebraicModel
            );

            staticAnalyzer.Initialize();
            staticAnalyzer.Solve();

            return loadControlAnalyzer.IncrementalDisplacementsLog;
        }
        private static void WriteToFile(string path, bool append, IncrementalDisplacementsLog results)
        {
            //if (append && !File.Exists(path)) append = false;
            //using (var writer = new StreamWriter(path, append))
            //{

            //    writer.WriteLine();
            //    writer.WriteLine("************************************************************************************************");

            //    var U = results.StoreDisplacementsPerIncrementInDoubleArray();


            //    foreach (var item in U)
            //    {
            //        writer.WriteLine(item);
            //    }
            //    writer.WriteLine("************************************************************************************************");
            //    writer.WriteLine();
            //}
        }
    }

}
