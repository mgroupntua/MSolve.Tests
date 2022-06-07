using System.Collections.Generic;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Numerics.Integration.Quadratures;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.FEM.Structural.Shells;
using MGroup.FEM.Structural.Embedding;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.Shells;
using MGroup.Constitutive.Structural.Cohesive;

namespace MGroup.FEM.Structural.Tests.ExampleModels
{
	public class Shell8andCohesiveNonLinearExample
	{
		public static Model CreateModel()
		{
			var model = new Model();

			model.SubdomainsDictionary.Add(key: 0, new Subdomain(id: 0));

			var Tk = 0.5;

			CreateNodes(model, Tk);
			CreateElements(model, Tk);
			ApplyBoundaryConditions(model);

			return model;
		}

		private static void ApplyBoundaryConditions(Model model)
		{
			var value_ext = 2 * 2.5 * 0.5;

			int[] points_with_negative_load;
			points_with_negative_load = new int[] { 1, 3, 6, 8 };
			int[] points_with_positive_load;
			points_with_positive_load = new int[] { 2, 4, 5, 7 };

			var constraints = new List<INodalDisplacementBoundaryCondition>();
			for (var i = 9; i < 17; i++)
			{
				constraints.Add(new NodalDisplacement(model.NodesDictionary[i], StructuralDof.TranslationX, amount: 0d));
				constraints.Add(new NodalDisplacement(model.NodesDictionary[i], StructuralDof.TranslationY, amount: 0d));
				constraints.Add(new NodalDisplacement(model.NodesDictionary[i], StructuralDof.TranslationZ, amount: 0d));
			}

			var loads = new List<INodalLoadBoundaryCondition>();
			for (var i = 0; i < 3; i++)
			{
				loads.Add(new NodalLoad
				(
					model.NodesDictionary[points_with_negative_load[i + 1]],
					StructuralDof.TranslationZ,
					amount: -0.3333333 * value_ext
				));

				loads.Add(new NodalLoad
				(
					model.NodesDictionary[points_with_positive_load[i + 1]],
					StructuralDof.TranslationZ,
					amount: 1.3333333 * value_ext
				));
			}

			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(constraints, loads));
		}

		private static void CreateElements(Model model, double Tk)
		{
			var VH = new double[8][];

			for (var i = 0; i < 8; i++)
			{
				VH[i] = new double[3];
				VH[i][0] = 0;
				VH[i][1] = 0;
				VH[i][2] = 1;
			}

			var Tk_vec = new double[8];
			for (var i = 0; i < 8; i++)
			{
				Tk_vec[i] = Tk;
			}

			var element1 = new Shell8NonLinear(
				new[]
				{
					model.NodesDictionary[8],
					model.NodesDictionary[3],
					model.NodesDictionary[1],
					model.NodesDictionary[6],
					model.NodesDictionary[5],
					model.NodesDictionary[2],
					model.NodesDictionary[4],
					model.NodesDictionary[7]
				},
				new ShellElasticMaterial3D(youngModulus: 1353000d, poissonRation: 0.3, shearCorrectionCoefficientK: 5 / 6d),
				GaussLegendre3D.GetQuadratureWithOrder(orderXi: 3, orderEta: 3, orderZeta: 3)
			)
			{
				ID = 1,
				oVn_i = VH,
				tk = Tk_vec
			};

			model.ElementsDictionary.Add(element1.ID, element1);
			model.SubdomainsDictionary[0].Elements.Add(element1);

			int[] coh_global_nodes;
			coh_global_nodes = new int[] { 8, 3, 1, 6, 5, 2, 4, 7, 16, 11, 9, 14, 13, 10, 12, 15 };

			var nodelist = new List<INode>();
			for (var i = 0; i < 16; i++)
			{
				nodelist.Add(model.NodesDictionary[coh_global_nodes[i]]);
			}

			var element2 = new CohesiveShell8ToHexa20(
				nodelist,
				new BenzeggaghKenaneCohesiveMaterial(to3: 57, do3: 5.7e-5, df3: 0.0098245610, to1: 57, do1: 5.7e-5, df1: 0.0098245610, nCurve: 1.4),
				GaussLegendre2D.GetQuadratureWithOrder(orderXi: 3, orderEta: 3)
			)
			{
				ID = 2,
				oVn_i = VH,
				tk = Tk_vec,
				ShellElementSide = 0
			};

			model.ElementsDictionary.Add(element2.ID, element2);
			model.SubdomainsDictionary[0].Elements.Add(element2);
		}

		private static void CreateNodes(Model model, double Tk)
		{
			var nodeID = 1;
			for (var i = 0; i < 3; i++)
			{
				model.NodesDictionary.Add(nodeID, new Node(id: nodeID, x: 0, y: i * 0.25, z: 0d));
				nodeID++;
			}

			for (var i = 0; i < 2; i++)
			{
				model.NodesDictionary.Add(nodeID, new Node(id: nodeID, x: 0.25, y: i * 0.5, z: 0d));
				nodeID++;
			}

			for (var i = 0; i < 3; i++)
			{
				model.NodesDictionary.Add(nodeID, new Node(id: nodeID, x: 0.5, y: i * 0.25, z: 0d));
				nodeID++;
			}

			for (var i = 0; i < 3; i++)
			{
				model.NodesDictionary.Add(nodeID, new Node(id: nodeID, x: 0, y: i * 0.25, z: -0.5 * Tk));
				nodeID++;
			}

			for (var i = 0; i < 2; i++)
			{
				model.NodesDictionary.Add(nodeID, new Node(id: nodeID, x: 0.25, y: i * 0.5, z: -0.5 * Tk));
				nodeID++;
			}

			for (var i = 0; i < 3; i++)
			{
				model.NodesDictionary.Add(nodeID, new Node(id: nodeID, x: 0.5, y: i * 0.25, z: -0.5 * Tk));
				nodeID++;
			}
		}

		public static IReadOnlyList<double[]> GetExpectedDisplacements()
		{
			var expectedDisplacements = new double[5][];

			expectedDisplacements[0] = new double[] {
				-1.501306714739351400e-05, 4.963733738129490800e-06, -1.780945407868029400e-05, -1.499214801866540600e-05, -5.822833969672272200e-05 };
			expectedDisplacements[1] = new double[] {
				-1.500991892603005000e-05, 4.962619842302796000e-06, -1.780557361553905700e-05, -1.498958552758854400e-05, -5.821676140520536400e-05 };
			expectedDisplacements[2] = new double[] {
				-3.001954880280401800e-05, 9.925100656477526600e-06, -3.561116405104391700e-05, -2.997946837566090700e-05, -1.164336113147322500e-04 };
			expectedDisplacements[3] = new double[] {
				-3.074327250558424700e-05, 1.064972618932890100e-05, -3.846410374898863100e-05, -3.069783728664514200e-05, -1.191612724600880000e-04 };
			expectedDisplacements[4] = new double[] {
				-3.074281618479765600e-05, 1.064926767853693300e-05, -3.846254167901110600e-05, -3.069737876082750600e-05, -1.191596225034872200e-04 };
			return expectedDisplacements;
		}
	}
}
