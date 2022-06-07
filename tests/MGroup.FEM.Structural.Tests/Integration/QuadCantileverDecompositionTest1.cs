using System.Collections.Generic;
using MGroup.FEM.Structural.Tests.Commons;
using MGroup.FEM.Structural.Tests.ExampleModels;
using Xunit;

namespace MGroup.FEM.Structural.Tests.Integration
{
	public class QuadCantileverDecompositionTest1
	{
		private static readonly Dictionary<int, int[]> expectedSubdomains = new Dictionary<int, int[]>()
		{
			{ 0, new int[] {0,4,1,5,8,9,2,6,10,12,13,14,3,7,11,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31}},
			{ 1, new int[] {32,36,33,37,40,41,34,38,42,44,45,46,35,39,43,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63}},
			{ 2, new int[] {64,68,65,69,72,73,66,70,74,76,77,78,67,71,75,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95}}
		};

		[Fact]
		private static void RunTest()
		{
			var model = QuadCantileverExample1.CreateModel();

			model.ConnectDataStructures();
			var domainDecomposer = new AutomaticDomainDecomposer(model, 3);
			domainDecomposer.UpdateModel();

			Utilities.CheckModelSubdomains(expectedSubdomains, model);
		}
	}
}
