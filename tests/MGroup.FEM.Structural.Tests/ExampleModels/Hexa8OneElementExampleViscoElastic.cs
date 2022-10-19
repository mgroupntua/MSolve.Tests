using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.Constitutive.Structural.Continuum;
using MGroup.Constitutive.Structural.Transient;
using MGroup.FEM.Structural.Continuum;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Entities;
using System.Collections.Generic;

namespace MGroup.FEM.Structural.Tests.ExampleModels
{
    public class Hexa8OneElementExampleViscoElastic
    {
        public static Model CreateModel()
        {
            var gnormi = new List<double>();
            var knormi = new List<double>();
            var taui = new List<double>();
            /*int iCountLinesOfFile = 0;
            using (TextReader reader = File.OpenText("C:\\Users\\DELL\\Source\\Repos\\dsavvas\\MSolve.Tests\\PEEK_Time_Prory.txt"))
            {
                CultureInfo usCulture = new CultureInfo("en-US");
                NumberFormatInfo dbNumberFormat = usCulture.NumberFormat;
                string text;
                while ((text = reader.ReadLine()) != null)
                {
                    string[] bits = text.Split('\t');
                    gnormi.Add(double.Parse(bits[0], dbNumberFormat));
                    knormi.Add(double.Parse(bits[1], dbNumberFormat));
                    taui.Add(double.Parse(bits[2], dbNumberFormat));
                    iCountLinesOfFile++;
                }
            }
            double[,] viscoData = new double[iCountLinesOfFile, 3];
            for (int i = 0; i < iCountLinesOfFile; i++)
            {
                viscoData[i, 0] = gnormi[i];
                viscoData[i, 1] = knormi[i];
                viscoData[i, 2] = taui[i];
            }*/

            var viscoData = new double[,]
            {
                {0.0293033,  0d,   4.85007E-07 },
                {0.051821,    0d,   6.7444E-06},
                {0.0787804,   0d,   5.70198E-05},
                {0.10191,     0d,   0.000387505},
                {0.11675,     0d,   0.00234837},
                {0.12073,     0d,   0.0134637},
                {0.11375,     0d,   0.0763074},
                {0.09836,     0d,   0.44479},
                {0.0786564,   0d,   2.7783},
                {0.0586492,   0d,   19.514},
                {0.0410194,   0d,   164.32},
                {0.0268582,   0d,   1834.1},
                {0.0168322,   0d,   33546 },
            };
            for (int i = 0; i < viscoData.Length / 3; i++)
            {
                gnormi.Add(viscoData[i, 0]);
                knormi.Add(viscoData[i, 2]);
                taui.Add(viscoData[i, 1]);
            };

            var nodeData = new double[,] {
                {1.0,1.0,1.0},
                {0.0,1.0,1.0},
                {0.0,0.0,1.0},
                {1.0,0.0,1.0},
                {1.0,1.0,0.0},
                {0.0,1.0,0.0},
                {0.0,0.0,0.0},
                {1.0,0.0,0.0}
            };

            var elementData = new int[,] {
                {1,1,2,3,4,5,6,7,8}
            };

            var model = new Model();

            model.SubdomainsDictionary.Add(key: 0, new Subdomain(id: 0));

            for (var i = 0; i < nodeData.GetLength(0); i++)
            {
                var nodeId = i + 1;
                model.NodesDictionary.Add(nodeId, new Node(
                    id: nodeId,
                    x: nodeData[i, 0],
                    y: nodeData[i, 1],
                    z: nodeData[i, 2]));
            }

            for (var i = 0; i < elementData.GetLength(0); i++)
            {
                var nodeSet = new Node[8];
                for (var j = 0; j < 8; j++)
                {
                    var nodeID = elementData[i, j + 1];
                    nodeSet[j] = (Node)model.NodesDictionary[nodeID];
                }

                var viscoMaterial = new ElasticMaterial3DVisco(800000, 0.34, "LONG_TERM", viscoData);
                viscoMaterial.SetCurrentTime(1d);
                var elementFactory = new ContinuumElement3DFactory(viscoMaterial, new TransientAnalysisProperties(1, 0, 0));
                var element = elementFactory.CreateElement(CellType.Hexa8, nodeSet);
                element.ID = i + 1;

                model.ElementsDictionary.Add(element.ID, element);
                model.SubdomainsDictionary[0].Elements.Add(element);
            }

            var constraints = new List<INodalDisplacementBoundaryCondition>();
            var idOfConstrainedNodes = new int[4] { 2, 3, 7, 6 };

            for (var i = 0; i < idOfConstrainedNodes.GetLength(0); i++)
            {
                var id = idOfConstrainedNodes[i];
                constraints.Add(new NodalDisplacement(model.NodesDictionary[id], StructuralDof.TranslationX, amount: 0d));
                constraints.Add(new NodalDisplacement(model.NodesDictionary[id], StructuralDof.TranslationY, amount: 0d));
                constraints.Add(new NodalDisplacement(model.NodesDictionary[id], StructuralDof.TranslationZ, amount: 0d));
            }

            var loads = new List<INodalLoadBoundaryCondition>();
            var idOfNodesWithLoads = new int[4] { 1, 4, 8, 5 };

            for (var i = 0; i < idOfNodesWithLoads.GetLength(0); i++)
            {
                var id = idOfNodesWithLoads[i];
                loads.Add(new NodalLoad(model.NodesDictionary[id], StructuralDof.TranslationX, amount: 3750d));
            }

            model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(constraints, loads));

            return model;
        }

        public static IReadOnlyList<double[]> GetExpectedDisplacements()
        {
            return new double[][]
            {
                new[] { 0.001086233672127, 0.001086233672127, 0.001086233672127, 0.001086233672127},
                new[] { 0.002172467344253, 0.002172467344253, 0.002172467344253, 0.002172467344253},
                new[] { 0.002323402149088, 0.002323402149088, 0.002323402149088, 0.002323402149088},
                new[] { 0.003409635821214, 0.003409635821214, 0.003409635821214, 0.003409635821214},
                new[] { 0.003634098595390, 0.003634098595390, 0.003634098595390, 0.003634098595390},
                new[] { 0.004720332267517, 0.004720332267517, 0.004720332267517, 0.004720332267517},
                new[] { 0.004995823064564, 0.004995823064564, 0.004995823064564, 0.004995823064564},
                new[] { 0.006082056736690, 0.006082056736690, 0.006082056736690, 0.006082056736690},
                new[] { 0.006397878688575, 0.006397878688575, 0.006397878688575, 0.006397878688575},
            };
        }
    }
}
