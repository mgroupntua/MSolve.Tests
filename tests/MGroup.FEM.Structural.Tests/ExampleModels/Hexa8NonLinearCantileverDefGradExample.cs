using System.Collections.Generic;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Numerics.Integration.Quadratures;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.Continuum;
using MGroup.FEM.Structural.Continuum;
using MGroup.MSolve.Numerics.Interpolation;

namespace MGroup.FEM.Structural.Tests.ExampleModels
{
	public class Hexa8NonLinearCantileverDefGradExample
	{
		private static readonly double[,] nodeData = new double[,] {
			{-0.250000,-0.250000,-1.000000},
			{0.250000,-0.250000,-1.000000},
			{-0.250000,0.250000,-1.000000},
			{0.250000,0.250000,-1.000000},
			{-0.250000,-0.250000,-0.500000},
			{0.250000,-0.250000,-0.500000},
			{-0.250000,0.250000,-0.500000},
			{0.250000,0.250000,-0.500000},
			{-0.250000,-0.250000,0.000000},
			{0.250000,-0.250000,0.000000},
			{-0.250000,0.250000,0.000000},
			{0.250000,0.250000,0.000000},
			{-0.250000,-0.250000,0.500000},
			{0.250000,-0.250000,0.500000},
			{-0.250000,0.250000,0.500000},
			{0.250000,0.250000,0.500000},
			{-0.250000,-0.250000,1.000000},
			{0.250000,-0.250000,1.000000},
			{-0.250000,0.250000,1.000000},
			{0.250000,0.250000,1.000000}
		};

		private static readonly int[,] elementData = new int[,] {
			{1,8,7,5,6,4,3,1,2},
			{2,12,11,9,10,8,7,5,6},
			{3,16,15,13,14,12,11,9,10},
			{4,20,19,17,18,16,15,13,14}
		};

		public static Model CreateModel()
		{
			
			var model = new Model();

			model.SubdomainsDictionary.Add(key: 0, new Subdomain(id: 0));

			for (var i = 0; i < nodeData.GetLength(0); i++)
			{
				var nodeId = i + 1;
				model.NodesDictionary.Add(nodeId, new Node(
					id: nodeId,
					x: nodeData[i, 0],
					y: nodeData[i, 1],
					z: nodeData[i, 2])
				);
			}

			for (var i = 0; i < elementData.GetLength(0); i++)
			{
				var nodeSet = new Node[8];
				for (var j = 0; j < 8; j++)
				{
					var nodeID = elementData[i, j + 1];
					nodeSet[j] = (Node)model.NodesDictionary[nodeID];
				}
				var element = new ContinuumElement3DNonLinearDefGrad(
					nodeSet,
					new ElasticMaterial3DDefGrad(youngModulus: 1353000, poissonRatio: 0.3),
					GaussLegendre3D.GetQuadratureWithOrder(orderXi: 3, orderEta: 3, orderZeta: 3),
					InterpolationHexa8.UniqueInstance
				)
				{
					ID = i + 1
				};

				model.ElementsDictionary.Add(element.ID, element);
				model.SubdomainsDictionary[0].Elements.Add(element);
			}

			var constraints = new List<INodalDisplacementBoundaryCondition>();
			for (var i = 1; i < 5; i++)
			{
				constraints.Add(new NodalDisplacement(model.NodesDictionary[i], StructuralDof.TranslationX, amount: 0d));
				constraints.Add(new NodalDisplacement(model.NodesDictionary[i], StructuralDof.TranslationY, amount: 0d));
				constraints.Add(new NodalDisplacement(model.NodesDictionary[i], StructuralDof.TranslationZ, amount: 0d));
			}

			var loads = new List<INodalLoadBoundaryCondition>();
			for (var i = 17; i < 21; i++)
			{
				loads.Add(new NodalLoad
				(
					model.NodesDictionary[i],
					StructuralDof.TranslationX,
					amount: 1 * 850d
				));
			}

			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(constraints, loads));

			return model;
		}

		public static IReadOnlyList<double[]> GetExpectedDisplacements()
		{
			var expectedDisplacements = new double[11][];

			expectedDisplacements[0] = new double[] {
				 0.039075524153873623, -0.032541895181220408, -0.057387148941853101, -0.071994381984550326,  -0.077053554770404833
			};

			expectedDisplacements[0] = new double[] {
				3.907552415387362300e-02, -3.254189518122040800e-02, -5.738714894185310100e-02, -7.199438198455032600e-02, -7.705355477040483300e-02 };
			expectedDisplacements[1] = new double[] {
				4.061313406968563400e-02, -3.418876666892714500e-02, -6.682708262609965400e-02, -9.647418428408424700e-02, -1.214556593711370000e-01 };
			expectedDisplacements[2] = new double[] {
				4.036171804663909300e-02, -3.396515033613205900e-02, -6.665084050819490600e-02, -9.713633946904017000e-02, -1.236631490430697600e-01 };
			expectedDisplacements[3] = new double[] {
				4.032905162001462800e-02, -3.393260905426281900e-02, -6.657423779424630200e-02, -9.701032579889114200e-02, -1.234941821043235900e-01 };
			expectedDisplacements[4] = new double[] {
				4.032900093364350700e-02, -3.393255831972321500e-02, -6.657411965268195100e-02, -9.701012513482368300e-02, -1.234939001150344400e-01 };
			expectedDisplacements[5] = new double[] {
				8.095088461395548400e-02, -6.826589092291023000e-02, -1.393261307096994000e-01, -2.129883579558797000e-01, -2.840192458274605800e-01 };
			expectedDisplacements[6] = new double[] {
				8.179065808895391600e-02, -6.914910025670165100e-02, -1.449912527358244700e-01, -2.283048858573358000e-01, -3.126785624370127000e-01 };
			expectedDisplacements[7] = new double[] {
				8.008398180684392400e-02, -6.747544383562544000e-02, -1.408463169597064000e-01, -2.210877012127209200e-01, -3.022981704019522300e-01 };
			expectedDisplacements[8] = new double[] {
				7.976397887674688300e-02, -6.715673915988762400e-02, -1.400151566610138300e-01, -2.195056794855129700e-01, -2.998365539162924900e-01 };
			expectedDisplacements[9] = new double[] {
				7.975945236918889600e-02, -6.715223199537226400e-02, -1.400036710136937400e-01, -2.194845023343510200e-01, -2.998046100841828000e-01 };
			expectedDisplacements[10] = new double[] {
				7.975944951878896600e-02, -6.715222916021290600e-02, -1.400036636464831200e-01, -2.194844883932760600e-01, -2.998045884933974200e-01 };

			return expectedDisplacements;
		}
	}
}
