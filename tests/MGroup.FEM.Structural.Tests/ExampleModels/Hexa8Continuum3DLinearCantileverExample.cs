using System.Collections.Generic;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.Constitutive.Structural.Continuum;
using MGroup.Constitutive.Structural.Transient;
using MGroup.FEM.Structural.Continuum;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Entities;


namespace MGroup.FEM.Structural.Tests.ExampleModels
{
	public class Hexa8Continuum3DLinearCantileverExample
	{
		public static Model CreateModel()
		{
			var nodeData = new double[,] {
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
			double correction = 10;// +20;

			var elementData = new int[,] {
				{1,8,7,5,6,4,3,1,2},
				{2,12,11,9,10,8,7,5,6},
				{3,16,15,13,14,12,11,9,10},
				{4,20,19,17,18,16,15,13,14}
			};

			var model = new Model();

			model.SubdomainsDictionary.Add(key: 0, new Subdomain(id: 0));

			for (var i = 0; i < nodeData.GetLength(0); i++)
			{
				var nodeId = i + 1;
				model.NodesDictionary.Add(nodeId, new Node(
					id: nodeId,
					x: nodeData[i, 0] + correction,
					y: nodeData[i, 1] + correction,
					z: nodeData[i, 2] + correction));
			}

			for (var i = 0; i < elementData.GetLength(0); i++)
			{
				var nodeSet = new Node[8];
				for (var j = 0; j < 8; j++)
				{
					var nodeID = elementData[i, j + 1];
					nodeSet[j] = (Node)model.NodesDictionary[nodeID];
				}

				var elementFactory = new ContinuumElement3DFactory(new VonMisesMaterial3D(1353000, 0.30, 119300, 0.15), new TransientAnalysisProperties(1, 0, 0));
				var element = elementFactory.CreateElement(CellType.Hexa8, nodeSet);
				element.ID = i + 1;

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
				loads.Add(new NodalLoad(model.NodesDictionary[i], StructuralDof.TranslationX, amount: 1 * 850d));
			}

			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(constraints, loads));

			return model;
		}

		public static IReadOnlyList<double[]> GetExpectedDisplacements()
		{
			return new double[][]
			{
				new[] { 0.03907552415387404, -0.03254189518122034, -0.05738714894185283, -0.07199438198455, -0.07705355477040472 },
				new[] { 0.07815104830774808, -0.06508379036244068, -0.11477429788370566, -0.1439887639691, -0.15410710954080944 },
				new[] { 0.08106195315153275, -0.06640555251140878, -0.11616127549599153, -0.14536326728872137, -0.15548356666530583 },
				new[] { 0.08110323961011925, -0.06641478802120071, -0.11617071896815707, -0.14537267098223042, -0.1554929765892069 },
				new[] { 0.08110397254602059, -0.06641490585564291, -0.11617083666001651, -0.14537278870136286, -0.1554930943040676 },
				new[] { 0.08110398686046946, -0.06641490813188103, -0.11617083892988608, -0.1453727909724505, -0.15549309657496455 },
				new[] { 0.081103987140787, -0.06641490817644792, -0.11617083897432437, -0.14537279101691347, -0.15549309661942362 },
			};
		}
	}
}
