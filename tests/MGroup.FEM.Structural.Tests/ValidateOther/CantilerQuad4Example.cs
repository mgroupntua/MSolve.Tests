using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Xml.Linq;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.Constitutive.Structural.Planar;
using MGroup.FEM.Structural.Continuum;
using MGroup.FEM.Structural.Tests.ExampleModels;
using MGroup.LinearAlgebra.Iterative.PreconditionedConjugateGradient;
using MGroup.LinearAlgebra.Iterative.Termination.Iterations;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Discretization.Meshes.Structured;
using MGroup.NumericalAnalyzers;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.Solvers.Direct;
using MGroup.Solvers.Iterative;
using Xunit;

namespace MGroup.FEM.Structural.Tests.ValidateOther
{
	public class CantilerQuad4Example
	{
		private static List<(INode node, IDofType dof)> watchDofs = new List<(INode node, IDofType dof)>();

		[Fact]
		public static void RunExample()
		{
			var model = CreateFemModel();
			var log = SolveModel(model);
			double topRightUy = log.DOFValues.Single().val;
			Debug.WriteLine("Finished");
		}

		private static Model CreateFemModel()
		{
			// Params
			int numNodesX = 21;
			int numNodesY = 5;
			double maxX = 2.4;
			double maxY = 0.4;
			double thickness = maxY;
			double E = 200E6;
			double v = 0.3;
			double totalP = 1000;
			double dx = maxX / (numNodesX - 1);
			double dy = maxY / (numNodesY - 1);

			var model = new Model();
			model.SubdomainsDictionary[0] = new Subdomain(0);

			// Mesh generation
			var mesh = new UniformCartesianMesh2D.Builder(
				new double[] { 0.0, 0.0 },
				new double[] { maxX, maxY },
				new int[] { numNodesX - 1, numNodesY - 1 })
				.SetMajorAxis(1)
				.BuildMesh();

			// Nodes
			foreach ((int nodeID, double[] coords) in mesh.EnumerateNodes())
			{
				model.NodesDictionary.Add(nodeID, new Node(nodeID, coords[0], coords[1], 0.0));
			}

			// Elements
			var elementFactory = new ContinuumElement2DFactory(
				commonThickness: thickness,
				new ElasticMaterial2D(youngModulus: E, poissonRatio: v, StressState2D.PlaneStress),
				commonDynamicProperties: null
			);
			foreach ((int elementID, int[] nodeIDs) in mesh.EnumerateElements())
			{
				int[] elemNodeIds = mesh.GetElementConnectivity(elementID);
				INode[] elemNodes = elemNodeIds.Select(id => model.NodesDictionary[id]).ToArray();
				ContinuumElement2D element = elementFactory.CreateElement(CellType.Quad4, elemNodes);
				
				model.ElementsDictionary.Add(elementID, element);
				model.SubdomainsDictionary[0].Elements.Add(element);
			}

			// Loads
			var loads = new List<NodalLoad>();
			List<Node> loadedNodes = FindsNodesWithX(maxX, model, mesh);
			double loadPerNode = totalP / loadedNodes.Count;
			foreach (Node node in loadedNodes)
			{
				loads.Add(new NodalLoad(node, StructuralDof.TranslationY, -loadPerNode));
			}

			// Supports
			var supports = new List<NodalDisplacement>();
			List<Node> supportedNodes = FindsNodesWithX(0.0, model, mesh);
			foreach (Node node in supportedNodes)
			{
				supports.Add(new NodalDisplacement(node, StructuralDof.TranslationX, 0.0));
				supports.Add(new NodalDisplacement(node, StructuralDof.TranslationY, 0.0));
			}

			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(supports, loads));

			// Monitor dofs
			Node topRightNode = FindsNodesWithXY(maxX, maxY, model, mesh).Single();
			watchDofs.Add((topRightNode, StructuralDof.TranslationY));

			return model;
		}

		private static DOFSLog SolveModel(Model model)
		{
			var solverFactory = new LdlSkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(algebraicModel);
			var problem = new ProblemStructural(model, algebraicModel);

			var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);
			var staticAnalyzer = new StaticAnalyzer(algebraicModel, problem, linearAnalyzer);

			linearAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

			staticAnalyzer.Initialize();
			staticAnalyzer.Solve();

			return (DOFSLog)linearAnalyzer.Logs[0];
		}


		private static List<Node> FindsNodesWithX(double targetX, Model model, UniformCartesianMesh2D mesh)
		{
			double dx = mesh.DistancesBetweenPoints[0];
			double tol = dx / 10.0;
			var result = new List<Node>();
			foreach (Node node in model.NodesDictionary.Values)
			{
				if (Math.Abs(node.X - targetX) < tol)
				{
					result.Add(node);
				}
			}
			return result;
		}

		private static List<Node> FindsNodesWithXY(double targetX, double targetY, Model model, UniformCartesianMesh2D mesh)
		{
			double dx = mesh.DistancesBetweenPoints[0];
			double dy = mesh.DistancesBetweenPoints[1];
			double tolX = dx / 10.0;
			double tolY = dy / 10.0;
			var result = new List<Node>();
			foreach (Node node in model.NodesDictionary.Values)
			{
				if (Math.Abs(node.X - targetX) < tolX && Math.Abs(node.Y - targetY) < tolY)
				{
					result.Add(node);
				}
			}
			return result;
		}

	}
}
