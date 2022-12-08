using System.Collections.Generic;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.Continuum;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.MSolve.Discretization.Entities;
using MGroup.FEM.Structural.Continuum;
using MGroup.MSolve.Numerics.Integration.Quadratures;
using MGroup.MSolve.Numerics.Interpolation;
using MGroup.Constitutive.Structural.Transient;
using MGroup.MSolve.Discretization;
using MGroup.Constitutive.Structural.InitialConditions;
using System;
using MGroup.MSolve.Discretization.BoundaryConditions;

namespace MGroup.FEM.Structural.Tests.ExampleModels
{
	public class Hexa8ContinuumNonLinearCantileverDynamicExample
	{
		public static IStructuralDofType monitoredDof = StructuralDof.TranslationX;
		public static int monitoredNode { get; private set; } = 17;

		public static double density = 1;
        public static double load_value = 1 * 100d;//initial value = 1 * 850d;

		private const double youngModulus = 1353000;
		private const double poissonRatio = 0.3;

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
					x: nodeData[i, 0],
					y: nodeData[i, 1],
					z: nodeData[i, 2]
				));
			}

			var factory = new ContinuumElement3DFactory(
				new ElasticMaterial3D(youngModulus: youngModulus, poissonRatio: poissonRatio),
				commonDynamicProperties: new TransientAnalysisProperties(density, 0, 0)
			);

			for (var i = 0; i < elementData.GetLength(0); i++)
			{
				var nodeSet = new Node[8];
				for (var j = 0; j < 8; j++)
				{
					var nodeID = elementData[i, j + 1];
					nodeSet[j] = (Node)model.NodesDictionary[nodeID];
				}

				var element = factory.CreateNonLinearElement(
					CellType.Hexa8,
					nodeSet,
					commonMaterial: new ElasticMaterial3D(youngModulus: youngModulus, poissonRatio: poissonRatio),
					commonDynamicProperties: new TransientAnalysisProperties(density, 0, 0));
				element.ID=	 i + 1;
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

			var emptyloads1 = new List<INodalLoadBoundaryCondition>();

			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(constraints, emptyloads1));

			return model;
		}

		public static void AddStaticNodalLoads(Model model)
        {
			var emptyConstraintsconstraints = new List<INodalDisplacementBoundaryCondition>();
			var loads = new List<INodalLoadBoundaryCondition>();
			for (var i = 17; i < 21; i++)
			{
				loads.Add(new NodalLoad(model.NodesDictionary[i], StructuralDof.TranslationX, amount: load_value));
			}

			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(emptyConstraintsconstraints, loads));
		}

		public static void AddInitialConditionsDisplacements(Model model)
		{
			double[,] intialNodalConditionValues = new double[,]
			{
				{0.000000,0.000000,0.000000},
				{0.000000,0.000000,0.000000},
				{0.000000,0.000000,0.000000},
				{0.000000,0.000000,0.000000},
				{0.019945,0.002829,0.015084},
				{0.018920,-0.002963,-0.016289},
				{0.019945,-0.002829,0.015084},
				{0.018920,0.002963,-0.016289},
				{0.066861,0.001258,0.023999},
				{0.063953,-0.001249,-0.029527},
				{0.066861,-0.001258,0.023999},
				{0.063953,0.001249,-0.029527},
				{0.130154,0.000629,0.026177},
				{0.125810,-0.000615,-0.039428},
				{0.130154,-0.000629,0.026177},
				{0.125810,0.000615,-0.039428},
				{0.200258,0.000252,0.023255},
				{0.195402,-0.000209,-0.046140},
				{0.200258,-0.000252,0.023255},
				{0.195402,0.000209,-0.046140},
				{0.195402,0.000209,-0.046140}
			};

			var emptyDomainInitialConditions = new List<IDomainStructuralInitialCondition>();
			var initialDisplacements = new List<INodalDisplacementInitialCondition>();

			for (var i = 5; i < 21; i++)
			{
				initialDisplacements.Add(new NodalInitialDisplacement(model.NodesDictionary[i], StructuralDof.TranslationX, amount: intialNodalConditionValues[i - 1, 0]));
				initialDisplacements.Add(new NodalInitialDisplacement(model.NodesDictionary[i], StructuralDof.TranslationY, amount: intialNodalConditionValues[i - 1, 1]));
				initialDisplacements.Add(new NodalInitialDisplacement(model.NodesDictionary[i], StructuralDof.TranslationZ, amount: intialNodalConditionValues[i - 1, 2]));
				
			}



			var modelInitialConditionsSets = new StructuralInitialConditionSet(initialDisplacements, emptyDomainInitialConditions);
			model.InitialConditions.Add(modelInitialConditionsSets);
		}

		public static void AddTransientLoadNoDelay(Model model)
		{
			var loads = new List<INodalLoadBoundaryCondition>();
			var emptyConstraints1 = new List<INodalDisplacementBoundaryCondition>();

			for (var i = 17; i < 21; i++)
			{
				loads.Add(new NodalLoad(model.NodesDictionary[i], StructuralDof.TranslationX, amount: load_value));
			}

			var boundaryLoadConditionSet = new StructuralBoundaryConditionSet(emptyConstraints1, loads);
			var transientLoadandBCs = new StructuralTransientBoundaryConditionSet(new List<IBoundaryConditionSet<IStructuralDofType>>() { boundaryLoadConditionSet }, TimeFunctionNoDelay);

			model.BoundaryConditions.Add(transientLoadandBCs);
		}


		public static double TimeFunctionWithDelay(double t, double amount)
		{
			double[] time = new double[] { 0,0.01,0.02,0.08 };

			double[] timeFunctionValues = new double[] { 0, 0, 1, 1 };

			for (int i1 = 0; i1 < time.Length - 1; i1++)
			{
				if (t == 0)
				{
					double endiameshTimeFuncValue = time[0];
					return endiameshTimeFuncValue * amount;
				}
				if ((time[i1] < t) && (t <= time[i1 + 1]))
				{
					double klisi = (timeFunctionValues[i1 + 1] - timeFunctionValues[i1]) / (time[i1 + 1] - time[i1]);
					double dt = t - time[i1];
					double deltaTimeFunction = klisi * dt;
					double endiameshTimeFuncValue = timeFunctionValues[i1] + deltaTimeFunction;
					return endiameshTimeFuncValue * amount;

				}

			}

			throw new Exception("time out of range");
		}

		public static double TimeFunctionNoDelay(double t, double amount)
		{
			if(t> 0.0105)
			{ var breakpoint = "here"; }
			double[] time = new double[] { 0, 0.01, 0.02, 0.08 };

			double[] timeFunctionValues = new double[] { 0, 1, 1, 1 };

			for (int i1 = 0; i1 < time.Length - 1; i1++)
			{
				if (t == 0)
				{
					double endiameshTimeFuncValue = time[0];
					return endiameshTimeFuncValue * amount;
				}
				if ((time[i1] < t) && (t <= time[i1 + 1]))
				{
					double klisi = (timeFunctionValues[i1 + 1] - timeFunctionValues[i1]) / (time[i1 + 1] - time[i1]);
					double dt = t - time[i1];
					double deltaTimeFunction = klisi * dt;
					double endiameshTimeFuncValue = timeFunctionValues[i1] + deltaTimeFunction;
					return endiameshTimeFuncValue * amount;

				}

			}

			throw new Exception("time out of range");
		}

		public static double TimeFunctionForData(double t, double amount)
		{
			double omega = 1;
			double amplitude = 1;
			if (t > 0.0105)
			{ var breakpoint = "here"; }
			double[] time = new double[] { 0, 0.01, 0.02, 0.08 };

			double[] timeFunctionValues = new double[] { 0, 1, 1, 1 };

			for (int i1 = 0; i1 < time.Length - 1; i1++)
			{
				if (t == 0)
				{
					double endiameshTimeFuncValue = time[0];
					return endiameshTimeFuncValue * amount;
				}
				if ((time[i1] < t) && (t <= time[i1 + 1]))
				{
					double klisi = (timeFunctionValues[i1 + 1] - timeFunctionValues[i1]) / (time[i1 + 1] - time[i1]);
					double dt = t - time[i1];
					double deltaTimeFunction = klisi * dt;
					double endiameshTimeFuncValue = timeFunctionValues[i1] + deltaTimeFunction;
					endiameshTimeFuncValue = amplitude*Math.Sin(omega * t);
					return endiameshTimeFuncValue * amount;

				}

			}

			throw new Exception("time out of range");
		}


		public static void AddTransientLoadWithDelay(Model model)
		{
			var loads = new List<INodalLoadBoundaryCondition>();
			var emptyConstraints1 = new List<INodalDisplacementBoundaryCondition>();

			for (var i = 17; i < 21; i++)
			{
				loads.Add(new NodalLoad(model.NodesDictionary[i], StructuralDof.TranslationX, amount: load_value));
			}

			var boundaryLoadConditionSet = new StructuralBoundaryConditionSet(emptyConstraints1, loads);
			var transientLoadandBCs = new StructuralTransientBoundaryConditionSet(
				new List<IBoundaryConditionSet<IStructuralDofType>>() { boundaryLoadConditionSet }, 
				TimeFunctionWithDelay);

			model.BoundaryConditions.Add(transientLoadandBCs);
		}

		public static double TimeFunctionPeriodicLoad(double t, double amount)
		{
			if (t > 0.0105)
			{ var breakpoint = "here"; }
			double[] time = new double[] { 0, 0.01, 0.03, 0.05, 0.07, 0.09, 0.11, 0.13, 0.15, 0.16 };

			double[] timeFunctionValues = new double[] { 0, 1, -1, 1, -1, 1, -1, 1, -1,0 };

			for (int i1 = 0; i1 < time.Length - 1; i1++)
			{
				if (t == 0)
				{
					double endiameshTimeFuncValue = time[0];
					return endiameshTimeFuncValue * amount;
				}
				if ((time[i1] < t) && (t <= time[i1 + 1]))
				{
					double klisi = (timeFunctionValues[i1 + 1] - timeFunctionValues[i1]) / (time[i1 + 1] - time[i1]);
					double dt = t - time[i1];
					double deltaTimeFunction = klisi * dt;
					double endiameshTimeFuncValue = timeFunctionValues[i1] + deltaTimeFunction;
					return endiameshTimeFuncValue * amount;

				}

			}

			throw new Exception("time out of range");
		}


		public static void AddPeriodicTransientLoad(Model model)
		{
			var loads = new List<INodalLoadBoundaryCondition>();
			var emptyConstraints1 = new List<INodalDisplacementBoundaryCondition>();

			for (var i = 17; i < 21; i++)
			{
				loads.Add(new NodalLoad(model.NodesDictionary[i], StructuralDof.TranslationX, amount: load_value));
			}

			var boundaryLoadConditionSet = new StructuralBoundaryConditionSet(emptyConstraints1, loads);
			var transientLoadandBCs = new StructuralTransientBoundaryConditionSet(
				new List<IBoundaryConditionSet<IStructuralDofType>>() { boundaryLoadConditionSet },
				TimeFunctionPeriodicLoad);

			model.BoundaryConditions.Add(transientLoadandBCs);
		}





		public static double[] GetExpectedDisplacementsSuddenLoad()
		{
			var expectedDisplacements = new double[]
			{
				0.00034026406619990207,
				0.0015601307030748278,
				0.0036519307803377483,
				0.0064078756345493024,
				0.0099027991319248984,
				0.014109459244222421,
				0.018737868006090376,
				0.023595256395184918,
				0.028801463177756351,
				0.034471459641864259,
				0.040438171457499311,
				0.0464937850205113,
				0.05270883453853345,
				0.059302861368140475,
				0.06634830125665156,
				0.073804850863422575,
				0.081726051725522941,
				0.090207653003384208,
				0.099166777710456872,
				0.10833418685250081,
				0.11745200958922609,
				0.12637504442798522,
				0.13500764332279025,
				0.14324681776011858,
				0.15099432374195804,
				0.15816351455066824,
				0.1646737939491289,
				0.170474583828909,
				0.17556491359047574,
				0.17998253278489121,
				0.18381705895720832,
				0.18723132343134524,
				0.19038446236108647,
				0.19329821539029898,
				0.19585401922544315,
				0.19792516331026275,
				0.19943069179756118,
				0.20025647278700809,
				0.20024396994309551,
				0.19931685984393249,
				0.19753016074859631,
				0.19493016203248781,
				0.19144428283650797,
				0.18698408581157869,
				0.18160126684934302,
				0.17547586378906235,
				0.16879915152202249,
				0.16173668023783164,
				0.15446755875783003,
				0.14716584746847511,
				0.13991248218145322,
				0.13265839831193246,
				0.12529721558855497,
				0.11777245238414974,
				0.11011383679684739,
				0.10237095445103599,
				0.094528090708638032,
				0.086528841624088582,
				0.078386912727900215,
				0.070194981078085777,
				0.062003205881473844,
				0.053808312093496492,
				0.045737322398767835,
				0.038122777248127994,
				0.03127104697393511,
				0.025239660023391484,
				0.019968591846117073,
				0.01551969769911519,
				0.011989695241238782,
				0.0092660668608494264,
				0.0071197440883747631,
				0.005527612510550021,
				0.0046514359417653148,
				0.0044739550367780129,
				0.0047251719316205847,
				0.0052491152473887435,
				0.00618473794892802,
				0.0076699021827224319,
				0.0096338791084117532,
				0.012061395096735863,
				0.015222744781370486,
				0.019400914704237795,
				0.024544788794942894,
				0.030415316471702938,
				0.036939498977843724,
				0.04417083004556821,
				0.051991380747438094,
				0.060128447588289731,
				0.068448349703995076,
				0.076982266427953527,
				0.085639262634750551,
				0.094127011777839109,
				0.10222588723220409,
				0.10996450064455277,
				0.11746453118171814,
				0.12477526437215097,
				0.13193855839687876,
				0.13907582620980649,
				0.14629054982595649,
				0.15354001313780027,
				0.1606724579028429,
				0.16754247237746966,
				0.17404506548205467,
				0.18009107390810727,
				0.18559428822953855,
				0.19045018532762453,
				0.19451230284811144,
				0.19762844962217546,
				0.19970932437173711,
				0.20073835871218959,
				0.20074777798066698,
				0.19985197463069163,
				0.19827801773970752,
				0.19625760969252118,
				0.1938678818326553,
				0.19105312209772271,
				0.18779145805497152,
				0.18412893168146219,
				0.18005038733469481,
				0.17545550253326442,
				0.17030519392935475,
				0.16467216489004888,
				0.15858148867893462,
				0.15190423567759084,
				0.14450362104530973,
				0.13642226680450317,
				0.12784081048096479,
				0.11892066699474858,
				0.10979513985430554,
				0.10066640327247028,
				0.091778745627046132,
				0.083269978627185021,
				0.075120009668469628,
				0.067277281033217673,
				0.059792746517184851,
				0.052788682505182122,
				0.046316901092360044,
				0.040302369465757493,
				0.034644172372233356,
				0.029322332911988273,
				0.024345999468361766,
				0.019640480159334034,
				0.015116746722387893,
				0.010878547251307727,
				0.0072249836128225782,
				0.004378857457892113,
				0.0023383407262267471,
				0.0010795868344601394,
				0.000743228804755797,
				0.0014537777531330558,
				0.0030876813467415423,
				0.0054464974267217393,
				0.00857094003140199,
				0.012618751438285293,
				0.017472350519746643,
				0.02275437888435838,
				0.028246049381469532,
				0.034009862731016791,
				0.040058010754289143,
				0.046212479585289977
			};


			return expectedDisplacements;
		}

		public static double[] GetExpectedDisplacementsSuddenLoadAndInitialConditionsDisplacements()
		{
			var expectedDisplacements = new double[]
			{
				//2.00258E-01, //comment out to msolve den epistrefei initial conditions san rssult gia time = 0;
				2.00048E-01,
				1.99230E-01,
				1.97656E-01,
				1.95402E-01,
				1.92513E-01,
				1.89010E-01,
				1.84907E-01,
				1.80218E-01,
				1.74954E-01,
				1.69140E-01,
				1.62832E-01,
				1.56110E-01,
				1.49051E-01,
				1.41696E-01,
				1.34046E-01,
				1.26107E-01,
				1.17923E-01,
				1.09564E-01,
				1.01096E-01,
				9.26021E-02,
				8.42050E-02,
				7.60202E-02,
				6.80801E-02,
				6.03580E-02,
				5.28809E-02,
				4.57554E-02,
				3.90689E-02,
				3.28409E-02,
				2.71058E-02,
				2.19573E-02,
				1.74478E-02,
				1.35148E-02,
				1.00905E-02,
				7.23179E-03,
				5.05234E-03,
				3.57596E-03,
				2.76451E-03,
				2.65203E-03,
				3.32238E-03,
				4.75741E-03,
				6.83374E-03,
				9.49543E-03,
				1.28181E-02,
				1.68619E-02,
				2.15644E-02,
				2.68345E-02,
				3.26525E-02,
				3.90014E-02,
				4.57718E-02,
				5.28307E-02,
				6.01469E-02,
				6.77678E-02,
				7.56884E-02,
				8.38182E-02,
				9.20675E-02,
				1.00391E-01,
				1.08736E-01,
				1.17013E-01,
				1.25143E-01,
				1.33108E-01,
				1.40914E-01,
				1.48525E-01,
				1.55848E-01,
				1.62768E-01,
				1.69190E-01,
				1.75052E-01,
				1.80318E-01,
				1.84971E-01,
				1.89012E-01,
				1.92448E-01,
				1.95265E-01,
				1.97413E-01,
				1.98831E-01,
				1.99502E-01,
				1.99464E-01,
				1.98766E-01,
				1.97427E-01,
				1.95445E-01,
				1.92826E-01,
				1.89571E-01,
				1.85650E-01,
				1.81024E-01,
				1.75714E-01,
				1.69818E-01,
				1.63451E-01,
				1.56673E-01,
				1.49509E-01,
				1.41998E-01,
				1.34204E-01,
				1.26171E-01,
				1.17933E-01,
				1.09571E-01,
				1.01215E-01,
				9.29685E-02,
				8.48403E-02,
				7.68041E-02,
				6.88882E-02,
				6.11719E-02,
				5.37089E-02,
				4.65220E-02,
				3.96815E-02,
				3.33179E-02,
				2.75206E-02,
				2.22737E-02,
				1.75369E-02,
				1.33518E-02,
				9.81427E-03,
				6.97120E-03,
				4.81560E-03,
				3.37381E-03,
				2.70984E-03,
				2.81983E-03,
				3.60007E-03,
				4.96613E-03,
				6.94594E-03,
				9.60415E-03,
				1.29283E-02,
				1.68582E-02,
				2.13846E-02,
				2.65289E-02,
				3.22378E-02,
				3.83904E-02,
				4.49239E-02,
				5.18737E-02,
				5.92630E-02,
				6.70231E-02,
				7.50571E-02,
				8.33121E-02,
				9.17308E-02,
				1.00189E-01,
				1.08557E-01,
				1.16800E-01,
				1.24945E-01,
				1.32969E-01,
				1.40780E-01,
				1.48288E-01,
				1.55440E-01,
				1.62180E-01,
				1.68444E-01,
				1.74196E-01,
				1.79455E-01,
				1.84232E-01,
				1.88496E-01,
				1.92179E-01,
				1.95210E-01,
				1.97525E-01,
				1.99083E-01,
				1.99876E-01,
				1.99926E-01,
				1.99261E-01,
				1.97904E-01,
				1.95862E-01,
				1.93123E-01,
				1.89662E-01,
				1.85498E-01,
				1.80719E-01,
				1.75428E-01,
				1.69687E-01,
				1.63519E-01,
				1.56953E-01, };
		    return expectedDisplacements;
		}

		public static double[] GetExpectedDisplacementsTransientLoadNoDelayADINA()
		{
			var expectedDisplacements = new double[]
			{
				0.00000E+00,
				1.70121E-05,
				9.49980E-05,
				2.77476E-04,
				5.97494E-04,
				1.09182E-03,
				1.79594E-03,
				2.73095E-03,
				3.90832E-03,
				5.34546E-03,
				7.06561E-03,
				9.08383E-03,
				1.14048E-02,
				1.40368E-02,
				1.69986E-02,
				2.03131E-02,
				2.40009E-02,
				2.80848E-02,
				3.25925E-02,
				3.75475E-02,
				4.29607E-02,
				4.88129E-02,
				5.50503E-02,
				6.16147E-02,
				6.84531E-02,
				7.55047E-02,
				8.27051E-02,
				9.00000E-02,
				9.73433E-02,
				1.04682E-01,
				1.11961E-01,
				1.19134E-01,
				1.26176E-01,
				1.33067E-01,
				1.39775E-01,
				1.46260E-01,
				1.52475E-01,
				1.58370E-01,
				1.63881E-01,
				1.68943E-01,
				1.73498E-01,
				1.77507E-01,
				1.80939E-01,
				1.83763E-01,
				1.85951E-01,
				1.87482E-01,
				1.88347E-01,
				1.88551E-01,
				1.88111E-01,
				1.87052E-01,
				1.85406E-01,
				1.83205E-01,
				1.80471E-01,
				1.77210E-01,
				1.73427E-01,
				1.69134E-01,
				1.64350E-01,
				1.59098E-01,
				1.53405E-01,
				1.47307E-01,
				1.40845E-01,
				1.34062E-01,
				1.27000E-01,
				1.19708E-01,
				1.12258E-01,
				1.04734E-01,
				9.72145E-02,
				8.97667E-02,
				8.24513E-02,
				7.53246E-02,
				6.84290E-02,
				6.17912E-02,
				5.54389E-02,
				4.94129E-02,
				4.37559E-02,
				3.84953E-02,
				3.36483E-02,
				2.92401E-02,
				2.53062E-02,
				2.18773E-02,
				1.89787E-02,
				1.66467E-02,
				1.49324E-02,
				1.38779E-02,
				1.34965E-02,
				1.37823E-02,
				1.47298E-02,
				1.63305E-02,
				1.85590E-02,
				2.13781E-02,
				2.47579E-02,
				2.86762E-02,
				3.30980E-02,
				3.79689E-02,
				4.32366E-02,
				4.88685E-02,
				5.48421E-02,
				6.11294E-02,
				6.77013E-02,
				7.45373E-02,
				8.16158E-02,
				8.88943E-02,
				9.63079E-02,
				1.03789E-01,
				1.11279E-01,
				1.18717E-01,
				1.26037E-01,
				1.33168E-01,
				1.40048E-01,
				1.46617E-01,
				1.52812E-01,
				1.58574E-01,
				1.63867E-01,
				1.68675E-01,
				1.72995E-01,
				1.76818E-01,
				1.80134E-01,
				1.82927E-01,
				1.85180E-01,
				1.86866E-01,
				1.87958E-01,
				1.88437E-01,
				1.88291E-01,
				1.87515E-01,
				1.86102E-01,
				1.84045E-01,
				1.81340E-01,
				1.78003E-01,
				1.74061E-01,
				1.69556E-01,
				1.64542E-01,
				1.59080E-01,
				1.53236E-01,
				1.47063E-01,
				1.40598E-01,
				1.33880E-01,
				1.26954E-01,
				1.19871E-01,
				1.12673E-01,
				1.05400E-01,
				9.80918E-02,
				9.07947E-02,
				8.35457E-02,
				7.63756E-02,
				6.93273E-02,
				6.24662E-02,
				5.58670E-02,
				4.95947E-02,
				4.37054E-02,
				3.82561E-02,
				3.33002E-02,
				2.88719E-02,
				2.49881E-02,
				2.16689E-02,
				1.89447E-02,
				1.68369E-02,
				1.53418E-02,
				1.44425E-02,
				1.41300E-02,
				1.44015E-02,
				
			};
		    return expectedDisplacements;
		}

		public static double[] GetExpectedDisplacementsPeriodicLoadADINA()
		{
			var expectedDisplacements = new double[]
			{
				 0.00000E+00,
				 1.70121E-05,
				 9.49980E-05,
				 2.77476E-04,
				 5.97494E-04,
				 1.09182E-03,
				 1.79594E-03,
				 2.73095E-03,
				 3.90832E-03,
				 5.34546E-03,
				 7.06561E-03,
				 9.08383E-03,
				 1.14048E-02,
				 1.40368E-02,
				 1.69986E-02,
				 2.03131E-02,
				 2.40009E-02,
				 2.80848E-02,
				 3.25925E-02,
				 3.75475E-02,
				 4.29607E-02,
				 4.87957E-02,
				 5.49543E-02,
				 6.13334E-02,
				 6.78464E-02,
				 7.43949E-02,
				 8.08790E-02,
				 8.72235E-02,
				 9.33705E-02,
				 9.92504E-02,
				 1.04784E-01,
				 1.09912E-01,
				 1.14605E-01,
				 1.18834E-01,
				 1.22550E-01,
				 1.25689E-01,
				 1.28185E-01,
				 1.29962E-01,
				 1.30931E-01,
				 1.31002E-01,
				 1.30110E-01,
				 1.28217E-01,
				 1.25307E-01,
				 1.21363E-01,
				 1.16379E-01,
				 1.10358E-01,
				 1.03323E-01,
				 9.53164E-02,
				 8.63885E-02,
				 7.66006E-02,
				 6.60191E-02,
				 5.47071E-02,
				 4.27085E-02,
				 3.00459E-02,
				 1.67387E-02,
				 2.82145E-03,
				-1.16573E-02,
				-2.66457E-02,
				-4.20841E-02,
				-5.78946E-02,
				-7.39855E-02,
				-9.02353E-02,
				-1.06477E-01,
				-1.22531E-01,
				-1.38216E-01,
				-1.53339E-01,
				-1.67729E-01,
				-1.81267E-01,
				-1.93859E-01,
				-2.05412E-01,
				-2.15845E-01,
				-2.25110E-01,
				-2.33176E-01,
				-2.39981E-01,
				-2.45440E-01,
				-2.49480E-01,
				-2.52039E-01,
				-2.53042E-01,
				-2.52396E-01,
				-2.50024E-01,
				-2.45883E-01,
				-2.39946E-01,
				-2.32183E-01,
				-2.22584E-01,
				-2.11186E-01,
				-1.98064E-01,
				-1.83299E-01,
				-1.66978E-01,
				-1.49211E-01,
				-1.30128E-01,
				-1.09851E-01,
				-8.84788E-02,
				-6.61109E-02,
				-4.28594E-02,
				-1.88362E-02,
				 5.86414E-03,
				 3.11543E-02,
				 5.69324E-02,
				 8.30786E-02,
				 1.09465E-01,
				 1.35951E-01,
				 1.62331E-01,
				 1.88308E-01,
				 2.13568E-01,
				 2.37826E-01,
				 2.60807E-01,
				 2.82251E-01,
				 3.01962E-01,
				 3.19808E-01,
				 3.35678E-01,
				 3.49468E-01,
				 3.61109E-01,
				 3.70577E-01,
				 3.77854E-01,
				 3.82886E-01,
				 3.85596E-01,
				 3.85906E-01,
				 3.83724E-01,
				 3.78930E-01,
				 3.71415E-01,
				 3.61132E-01,
				 3.48095E-01,
				 3.32354E-01,
				 3.13989E-01,
				 2.93126E-01,
				 2.69932E-01,
				 2.44591E-01,
				 2.17294E-01,
				 1.88256E-01,
				 1.57721E-01,
				 1.25931E-01,
				 9.31125E-02,
				 5.94771E-02,
				 2.52256E-02,
				-9.46180E-03,
				-4.44201E-02,
				-7.94780E-02,
				-1.14444E-01,
				-1.49120E-01,
				-1.83312E-01,
				-2.16812E-01,
				-2.49352E-01,
				-2.80613E-01,
				-3.10295E-01,
				-3.38126E-01,
				-3.63824E-01,
				-3.87139E-01,
				-4.07907E-01,
				-4.26049E-01,
				-4.41509E-01,
				-4.54229E-01,
				-4.64195E-01,
				-4.71432E-01,
				-4.75942E-01,
				-4.77665E-01,
				-4.76521E-01,
				-4.72452E-01,
				-4.65399E-01,
				-4.55280E-01,
				-4.42020E-01,
				-4.25606E-01,
				-4.06081E-01,
				-3.83505E-01,
				-3.57949E-01,
				-3.29532E-01,
				-2.98432E-01,
				-2.64843E-01,
				-2.28962E-01,
				-1.91003E-01,
				-1.51224E-01,
				-1.09906E-01,
				-6.73164E-02,
				-2.37132E-02,
				 2.06178E-02,
				 6.53736E-02,
				 1.10275E-01,
				 1.55076E-01,
				 1.99535E-01,
				 2.43396E-01,
				 2.86406E-01,
				 3.28315E-01,
				 3.68800E-01,
				 4.07423E-01,
				 4.43715E-01,
				 4.77252E-01,
				 5.07643E-01,
				 5.34546E-01,
				 5.57726E-01,
				 5.77086E-01,
				 5.92601E-01,
				 6.04270E-01,
				 6.12130E-01,
				 6.16251E-01,
				 6.16682E-01,
				 6.13384E-01,
				 6.06267E-01,
				 5.95242E-01,
				 5.80216E-01,
				 5.61071E-01,
				 5.37723E-01,
				 5.10221E-01,
				 4.78748E-01,
				 4.43559E-01,
				 4.04935E-01,
				 3.63221E-01,
				 3.18817E-01,
				 2.72108E-01,
				 2.23434E-01,
				 1.73138E-01,
				 1.21602E-01,
				 6.92179E-02,
				 1.63541E-02,
				-3.66319E-02,
				-8.93683E-02,
				-1.41486E-01,
				-1.92657E-01,
				-2.42597E-01,
				-2.91012E-01,
				-3.37603E-01,
				-3.82095E-01,
				-4.24234E-01,
				-4.63731E-01,
				-5.00239E-01,
				-5.33446E-01,
				-5.63112E-01,
				-5.89036E-01,
				-6.11048E-01,
				-6.29069E-01,
				-6.43129E-01,
				-6.53309E-01,
				-6.59675E-01,
				-6.62280E-01,
				-6.61186E-01,
				-6.56429E-01,
				-6.47965E-01,
				-6.35682E-01,
				-6.19457E-01,
				-5.99181E-01,
				-5.74759E-01,
				-5.46114E-01,
				-5.13251E-01,
				-4.76280E-01,
				-4.35393E-01,
				-3.90820E-01,
				-3.42848E-01,
				-2.91818E-01,
				-2.38108E-01,
				-1.82098E-01,
				-1.24174E-01,
				-6.47552E-02,
				-4.27995E-03,
				 5.68314E-02,
				 1.18175E-01,
				 1.79333E-01,
				 2.39879E-01,
				 2.99418E-01,
				 3.57592E-01,
				 4.14035E-01,
				 4.68360E-01,
				 5.20172E-01,
				 5.69091E-01,
				 6.14671E-01,
				 6.56361E-01,
				 6.93623E-01,
				 7.26038E-01,
				 7.53290E-01,
				 7.75167E-01,
				 7.91621E-01,
				 8.02786E-01,
				 8.08879E-01,
				 8.10088E-01,
				 8.06549E-01,
				 7.98363E-01,
				 7.85557E-01,
				 7.68026E-01,
				 7.45580E-01,
				 7.18055E-01,
				 6.85356E-01,
				 6.47449E-01,
				 6.04393E-01,
				 5.56426E-01,
				 5.03969E-01,
				 4.47546E-01,
				 3.87705E-01,
				 3.25027E-01,
				 2.60138E-01,
				 1.93650E-01,
				 1.26105E-01,
				 5.79982E-02,
				-1.01536E-02,
				-7.78129E-02,
				-1.44479E-01,
				-2.09723E-01,
				-2.73151E-01,
				-3.34361E-01,
				-3.92980E-01,
				-4.48717E-01,
				-5.01309E-01,
				-5.50459E-01,
				-5.95847E-01,
				-6.37200E-01,
				-6.74286E-01,
				-7.06866E-01,
				-7.34742E-01,
				-7.57823E-01,
				-7.76122E-01,
				-7.89706E-01,
				-7.98667E-01,
				-8.03145E-01,
				-8.03298E-01,
				-7.99228E-01,
				-7.90950E-01,
				-7.78421E-01,
				-7.61550E-01,
				-7.40182E-01,
				-7.14121E-01,
				-6.83186E-01,
				-6.47265E-01,
				-6.06309E-01,
				-5.60351E-01,
			

			};
		    return expectedDisplacements;
		}

		public static double[] GetExpectedDisplacementsTransientLoadWithDelayADINA()
		{
			var expectedDisplacements = new double[]
			{
				0.00000E+00,
				0.00000E+00,
				0.00000E+00,
				0.00000E+00,
				0.00000E+00,
				0.00000E+00,
				0.00000E+00,
				0.00000E+00,
				0.00000E+00,
				0.00000E+00,
				0.00000E+00,
				0.00000E+00,
				0.00000E+00,
				0.00000E+00,
				0.00000E+00,
				0.00000E+00,
				0.00000E+00,
				0.00000E+00,
				0.00000E+00,
				0.00000E+00,
				0.00000E+00,
				1.70121E-05,
				9.49980E-05,
				2.77476E-04,
				5.97494E-04,
				1.09182E-03,
				1.79594E-03,
				2.73095E-03,
				3.90832E-03,
				5.34546E-03,
				7.06561E-03,
				9.08383E-03,
				1.14048E-02,
				1.40368E-02,
				1.69986E-02,
				2.03131E-02,
				2.40009E-02,
				2.80848E-02,
				3.25925E-02,
				3.75475E-02,
				4.29607E-02,
				4.88129E-02,
				5.50503E-02,
				6.16147E-02,
				6.84531E-02,
				7.55047E-02,
				8.27051E-02,
				9.00000E-02,
				9.73433E-02,
				1.04682E-01,
				1.11961E-01,
				1.19134E-01,
				1.26176E-01,
				1.33067E-01,
				1.39775E-01,
				1.46260E-01,
				1.52475E-01,
				1.58370E-01,
				1.63881E-01,
				1.68943E-01,
				1.73498E-01,
				1.77507E-01,
				1.80939E-01,
				1.83763E-01,
				1.85951E-01,
				1.87482E-01,
				1.88347E-01,
				1.88551E-01,
				1.88111E-01,
				1.87052E-01,
				1.85406E-01,
				1.83205E-01,
				1.80471E-01,
				1.77210E-01,
				1.73427E-01,
				1.69134E-01,
				1.64350E-01,
				1.59098E-01,
				1.53405E-01,
				1.47307E-01,
				1.40845E-01,
				1.34062E-01,
				1.27000E-01,
				1.19708E-01,
				1.12258E-01,
				1.04734E-01,
				9.72145E-02,
				8.97667E-02,
				8.24513E-02,
				7.53246E-02,
				6.84290E-02,
				6.17912E-02,
				5.54389E-02,
				4.94129E-02,
				4.37559E-02,
				3.84953E-02,
				3.36483E-02,
				2.92401E-02,
				2.53062E-02,
				2.18773E-02,
				1.89787E-02,
				1.66467E-02,
				1.49324E-02,
				1.38779E-02,
				1.34965E-02,
				1.37823E-02,
				1.47298E-02,
				1.63305E-02,
				1.85590E-02,
				2.13781E-02,
				2.47579E-02,
				2.86762E-02,
				3.30980E-02,
				3.79689E-02,
				4.32366E-02,
				4.88685E-02,
				5.48421E-02,
				6.11294E-02,
				6.77013E-02,
				7.45373E-02,
				8.16158E-02,
				8.88943E-02,
				9.63079E-02,
				1.03789E-01,
				1.11279E-01,
				1.18717E-01,
				1.26037E-01,
				1.33168E-01,
				1.40048E-01,
				1.46617E-01,
				1.52812E-01,
				1.58574E-01,
				1.63867E-01,
				1.68675E-01,
				1.72995E-01,
				1.76818E-01,
				1.80134E-01,
				1.82927E-01,
				1.85180E-01,
				1.86866E-01,
				1.87958E-01,
				1.88437E-01,
				1.88291E-01,
				1.87515E-01,
				1.86102E-01,
				1.84045E-01,
				1.81340E-01,
				1.78003E-01,
				1.74061E-01,
				1.69556E-01,
				1.64542E-01,
				1.59080E-01,
				1.53236E-01,
				1.47063E-01,
				1.40598E-01,
				1.33880E-01,
				1.26954E-01,
				1.19871E-01,
				1.12673E-01,
				1.05400E-01,
				
			};
			return expectedDisplacements;
		}

	}
}
