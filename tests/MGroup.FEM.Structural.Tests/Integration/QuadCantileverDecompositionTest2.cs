using System.Collections.Generic;
using MGroup.FEM.Structural.Tests.ExampleModels;
using Xunit;
using MGroup.FEM.Structural.Tests.Commons;

namespace MGroup.FEM.Structural.Tests.Integration
{
	public class QuadCantileverDecompositionTest2
	{
		private static readonly Dictionary<int, int[]> expectedSubdomains = new Dictionary<int, int[]>()
		{
			{ 0, new int[] {0,4,1,5,8,9,2,6,10,12,13,14}},
			{ 1, new int[] {3,7,11,15,18,19,17,21,22,23,16,20}},
			{ 2, new int[] {24,28,25,29,32,33,26,30,34,36,37,38}},
			{ 3, new int[] {27,31,35,39,42,43,41,45,46,47,40,44}},
			{ 4, new int[] {48,52,49,53,56,57,50,54,58,60,61,62}},
			{ 5, new int[] {51,55,59,63,66,67,65,69,70,71,64,68}},
			{ 6, new int[] {72,76,73,77,80,81,74,78,82,84,85,86}},
			{ 7, new int[] {75,79,83,87,90,91,89,93,94,95,88,92}}
		};

		[Fact]
		private static void RunTest()
		{
			var model = QuadCantileverExample2.CreateModel();

			model.ConnectDataStructures();
			var domainDecomposer = new AutomaticDomainDecomposer(model, 8);
			domainDecomposer.UpdateModel();

			Utilities.CheckModelSubdomains(expectedSubdomains, model);
		}
	}
}
