using System.Collections.Generic;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Numerics.Integration.Quadratures;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.FEM.Structural.Shells;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.Shells;

namespace MGroup.FEM.Structural.Tests.ExampleModels
{
	public class Shell8NonLinearPlateExample
	{
		private static readonly double[,] nodeData =
			new double[,] {
				{10.000000,10.000000,0.000000},
				{7.500000,10.000000,0.000000},
				{5.000000,10.000000,0.000000},
				{2.500000,10.000000,0.000000},
				{0.000000,10.000000,0.000000},
				{10.000000,7.500000,0.000000},
				{7.500000,7.500000,0.000000},
				{5.000000,7.500000,0.000000},
				{2.500000,7.500000,0.000000},
				{0.000000,7.500000,0.000000},
				{10.000000,5.000000,0.000000},
				{7.500000,5.000000,0.000000},
				{5.000000,5.000000,0.000000},
				{2.500000,5.000000,0.000000},
				{0.000000,5.000000,0.000000},
				{10.000000,2.500000,0.000000},
				{7.500000,2.500000,0.000000},
				{5.000000,2.500000,0.000000},
				{2.500000,2.500000,0.000000},
				{0.000000,2.500000,0.000000},
				{10.000000,0.000000,0.000000},
				{7.500000,0.000000,0.000000},
				{5.000000,0.000000,0.000000},
				{2.500000,0.000000,0.000000},
				{0.000000,0.000000,0.000000},
				{8.750000,10.000000,0.000000},
				{6.250000,10.000000,0.000000},
				{3.750000,10.000000,0.000000},
				{1.250000,10.000000,0.000000},
				{8.750000,7.500000,0.000000},
				{6.250000,7.500000,0.000000},
				{3.750000,7.500000,0.000000},
				{1.250000,7.500000,0.000000},
				{8.750000,5.000000,0.000000},
				{6.250000,5.000000,0.000000},
				{3.750000,5.000000,0.000000},
				{1.250000,5.000000,0.000000},
				{8.750000,2.500000,0.000000},
				{6.250000,2.500000,0.000000},
				{3.750000,2.500000,0.000000},
				{1.250000,2.500000,0.000000},
				{8.750000,0.000000,0.000000},
				{6.250000,0.000000,0.000000},
				{3.750000,0.000000,0.000000},
				{1.250000,0.000000,0.000000},
				{10.000000,8.750000,0.000000},
				{10.000000,6.250000,0.000000},
				{10.000000,3.750000,0.000000},
				{10.000000,1.250000,0.000000},
				{7.500000,8.750000,0.000000},
				{7.500000,6.250000,0.000000},
				{7.500000,3.750000,0.000000},
				{7.500000,1.250000,0.000000},
				{5.000000,8.750000,0.000000},
				{5.000000,6.250000,0.000000},
				{5.000000,3.750000,0.000000},
				{5.000000,1.250000,0.000000},
				{2.500000,8.750000,0.000000},
				{2.500000,6.250000,0.000000},
				{2.500000,3.750000,0.000000},
				{2.500000,1.250000,0.000000},
				{0.000000,8.750000,0.000000},
				{0.000000,6.250000,0.000000},
				{0.000000,3.750000,0.000000},
				{0.000000,1.250000,0.000000}
			};

		private static readonly int[,] elementData =
			new int[,] {
				{1,1,2,7,6,26,50,30,46},
				{2,2,3,8,7,27,54,31,50},
				{3,3,4,9,8,28,58,32,54},
				{4,4,5,10,9,29,62,33,58},
				{5,6,7,12,11,30,51,34,47},
				{6,7,8,13,12,31,55,35,51},
				{7,8,9,14,13,32,59,36,55},
				{8,9,10,15,14,33,63,37,59},
				{9,11,12,17,16,34,52,38,48},
				{10,12,13,18,17,35,56,39,52},
				{11,13,14,19,18,36,60,40,56},
				{12,14,15,20,19,37,64,41,60},
				{13,16,17,22,21,38,53,42,49},
				{14,17,18,23,22,39,57,43,53},
				{15,18,19,24,23,40,61,44,57},
				{16,19,20,25,24,41,65,45,61}
			};
		public static Model CreateModel()
		{
			var model = new Model();

			model.SubdomainsDictionary.Add(key: 0, new Subdomain(id: 0));

			for (var i = 0; i < nodeData.GetLength(0); i++)
			{
				model.NodesDictionary.Add(i + 1, new Node(id: i + 1, x: nodeData[i, 0], y: nodeData[i, 1], z: nodeData[i, 2]));
			}

			var tk_shell_plate = 0.5;
			for (var i = 0; i < elementData.GetLength(0); i++)
			{
				var element = new Shell8NonLinear(
					new[]
					{
						model.NodesDictionary[elementData[i, 0 + 1]],
						model.NodesDictionary[elementData[i, 1 + 1]],
						model.NodesDictionary[elementData[i, 2 + 1]],
						model.NodesDictionary[elementData[i, 3 + 1]],
						model.NodesDictionary[elementData[i, 4 + 1]],
						model.NodesDictionary[elementData[i, 5 + 1]],
						model.NodesDictionary[elementData[i, 6 + 1]],
						model.NodesDictionary[elementData[i, 7 + 1]]
					},
					new ShellElasticMaterial3D(youngModulus: 135300d, poissonRation: 0.3, shearCorrectionCoefficientK: 5 / 6d),
					GaussLegendre3D.GetQuadratureWithOrder(orderXi: 3, orderEta: 3, orderZeta: 2)
				)
				{
					ID = i + 1,
					oVn_i = new double[][] {
						new double[] { 0,0,1 },
						new double[] { 0,0,1 },
						new double[] { 0,0,1 },
						new double[] { 0,0,1 },
						new double[] { 0,0,1 },
						new double[] { 0,0,1 },
						new double[] { 0,0,1 },
						new double[] { 0,0,1 }
					},
					tk = new double[] {
						tk_shell_plate,
						tk_shell_plate,
						tk_shell_plate,
						tk_shell_plate,
						tk_shell_plate,
						tk_shell_plate,
						tk_shell_plate,
						tk_shell_plate
					}
				};

				model.ElementsDictionary.Add(element.ID, element);
				model.SubdomainsDictionary[0].Elements.Add(element);
			}

			var cnstrnd = new int[] { 21, 22, 23, 24, 25, 26, 27, 28, 29, 1, 2, 3, 4, 5, 42, 43, 44, 45, 46, 47, 48, 49, 6, 11, 16, 10, 15, 20, 62, 63, 64, 65 };
			var constraints = new List<INodalDisplacementBoundaryCondition>();
			for (var i = 0; i < cnstrnd.GetLength(0); i++)
			{
				constraints.Add(new NodalDisplacement(model.NodesDictionary[cnstrnd[i]], StructuralDof.TranslationX, amount: 0d));
				constraints.Add(new NodalDisplacement(model.NodesDictionary[cnstrnd[i]], StructuralDof.TranslationY, amount: 0d));
				constraints.Add(new NodalDisplacement(model.NodesDictionary[cnstrnd[i]], StructuralDof.TranslationZ, amount: 0d));
				constraints.Add(new NodalDisplacement(model.NodesDictionary[cnstrnd[i]], StructuralDof.RotationX, amount: 0d));
				constraints.Add(new NodalDisplacement(model.NodesDictionary[cnstrnd[i]], StructuralDof.RotationY, amount: 0d));
			}

			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(
				constraints,
				new[]
				{
					new NodalLoad(model.NodesDictionary[13],    StructuralDof.TranslationZ, amount: 1 * 1d)
				}
			));

			return model;
		}

		public static IReadOnlyList<double[]> GetExpectedDisplacements()
		{
			var expectedDisplacements = new double[4][];

			// MIKRH AKRIVEIA TWN GAUSS POINT
			//expectedDisplacements[0] = new Dictionary<int, double> {
			//{ 0,1.341690670391555900e-21 }, {11,5.426715620119622300e-22 }, {23,3.510351163876216100e-19 }, {35,6.417697126233693400e-23 }, {47,9.703818759449350300e-06 }};
			//expectedDisplacements[1] = new Dictionary<int, double> {
			//{ 0,-1.766631410417230800e-10 }, {11,-1.766623168621910600e-10 }, {23,-1.384381502841035000e-16 }, {35,6.203787968704410500e-17 }, {47,9.703818433005175000e-06 }};
			//expectedDisplacements[2] = new Dictionary<int, double> {
			//{ 0,-5.299892610611580200e-10 }, {11,-5.299884368826366100e-10 }, {23,-1.389870365102331700e-16 }, {35,6.203729982771704200e-17 }, {47,1.940763607980495300e-05 }};
			//expectedDisplacements[3] = new Dictionary<int, double> {
			//{ 0,-7.066521335188173400e-10 }, {11,-7.066513887514916600e-10 }, {23,6.902764393184828500e-16 }, {35,-6.400975456584840300e-20 }, {47,1.940763490956924100e-05 }};

			// MEGALH AKRIVEIA TWN GAUSS POINT
			expectedDisplacements[0] = new double[] {
				-2.609907246226515400e-22, 1.413528855321975800e-22, -2.458757134937532800e-19, 2.289334051771179900e-22, 9.703818759449467200e-06 };
			expectedDisplacements[1] = new double[] {
				-1.766635527587974300e-10, -1.766634689918627600e-10, 9.157257237104792300e-17, -2.559311444145733000e-16, 9.703818432907000400e-06 };
			expectedDisplacements[2] = new double[] {
				-5.299896727797873100e-10, -5.299895890111070100e-10, 9.124758457251682500e-17, -2.559323845353319000e-16, 1.940763607970688300e-05 };
			expectedDisplacements[3] = new double[] {
				-7.066535263910381200e-10, -7.066531664241640700e-10, 4.128219398586412200e-16, 2.340064775305142000e-18, 1.940763490936303600e-05 };

			return expectedDisplacements;
		}
	}
}
