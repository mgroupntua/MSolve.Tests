﻿using ConvectionDiffusionTest;
using MGroup.Constitutive.ConvectionDiffusion;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Entities;
using MGroup.NumericalAnalyzers.Dynamic;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.NumericalAnalyzers;
using MGroup.Solvers.Direct;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Xunit;
using MGroup.FEM.ConvectionDiffusion.Tests.Commons;

namespace MGroup.FEM.ConvectionDiffusion.Tests.Integration
{
    public class ConvDiffProdDynamic2D
    {
        [Fact]
        private void RunTest()
        {
            double[] convectionCoeff = new[] { 1d, 1d };
            double diffusionCoeff = 1d;
            double capacityCoeff = 1d;
            double dependentSourceCoeff = 1d;
            double independentSourceCoeff = 1d;
            //                                  [1,1]                [1,2]             [2,1]              [2,2]
            double[] prescribedSolution = { 356.6310547980037, 356.6310547980037, 467.6967659230567, 467.69676592305683 };
            double tolerance = 1E-3;

            var model = Comsol2D9Quad.CreateModel(capacityCoeff, diffusionCoeff, convectionCoeff, dependentSourceCoeff, independentSourceCoeff);
            var solverFactory = new DenseMatrixSolver.Factory() { IsMatrixPositiveDefinite = false };
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemConvectionDiffusion(model, algebraicModel);

            var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);
            
            var dynamicAnalyzerBuilder = new BDFDynamicAnalyzer.Builder(algebraicModel, problem, linearAnalyzer, timeStep: 0.1, totalTime: 10, bdfOrder: 5);
            var dynamicAnalyzer = dynamicAnalyzerBuilder.Build();

            var watchDofs = new List<(INode node, IDofType dof)>()
            {
                (model.NodesDictionary[6], ConvectionDiffusionDof.UnknownVariable),  //[1,1]
                (model.NodesDictionary[10], ConvectionDiffusionDof.UnknownVariable), //[1,2]
                (model.NodesDictionary[7], ConvectionDiffusionDof.UnknownVariable),  //[2,1]
                (model.NodesDictionary[11], ConvectionDiffusionDof.UnknownVariable)  //[2,2]
            };

            linearAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

            dynamicAnalyzer.Initialize();
            dynamicAnalyzer.Solve();

            DOFSLog log = (DOFSLog)linearAnalyzer.Logs[0];
            var numericalSolution = new double[watchDofs.Count];
            for (int i = 0; i < numericalSolution.Length; i++)
            {
                numericalSolution[i] = log.DOFValues[watchDofs[i].node, watchDofs[i].dof];
            }
            Assert.True(ResultChecker.CheckResults(numericalSolution, prescribedSolution, tolerance));
        }
    }
}
