using System;
using System.Collections.Generic;
using System.Linq;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Numerics.Integration.Quadratures;
using MGroup.FEM.Structural.Continuum;
using MGroup.FEM.Structural.Embedding;
using MGroup.FEM.Structural.Shells;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.Continuum;
using MGroup.Constitutive.Structural.Shells;
using MGroup.Constitutive.Structural.Cohesive;
using MGroup.Constitutive.Structural.BoundaryConditions;

namespace MGroup.FEM.Structural.Tests.ExampleModels
{
	public class EmbeddingNLRVEExample
	{
		private const int subdomainID = 0;

		public static Model CreateModel()
		{
			double[,] Dq;
			Tuple<rveMatrixParameters, grapheneSheetParameters> mpgp;
			rveMatrixParameters mp;
			grapheneSheetParameters gp;
			var graphene_sheets_number = 1;

			int[] renumbering_vector =
			{30,1,4,5,7,11,
			12,8,13,14,15,17,
			18,24,31,25,32,33,
			16,19,20,26,34,35,
			27,36,37,2,3,9,
			6,10,21,38,28,22,
			29,23,39,40,41,42,
			43,44,45,46,47,48,
			49,50,51,52,53,54,
			55,56,57,58,59,60,
			61,62,63,64,65,66, };

			double[] Fxk_p_komvoi_rve = new double[]
			{-20,0,0,0,0,0,
			 20,0,0,-40,0,0,
			 0,0,0,40,0,0,
			 -20,0,0,0,0,0,
			 20,0,0,-40,0,0,
			 0,0,0,40,0,0,
			 -80,0,0,80,0,0,
			 -40,0,0,0,0,0,
			 40,0,0,-20,0,0,
			 0,0,0,20,0,0,
			 -40,0,0,0,0,0,
			 40,0,0,-20,0,0,
			 0,0,0,20,0,0, };

			double[][] o_xsunol = new double[graphene_sheets_number][];
			o_xsunol[0] = new double[]
			{-5.00000000000000270000,-25.98076211353316000000,-9.99999999999999820000,-0.43301270189221930000,-0.24999999999999994000,0.86602540378443871000,
			-15.00000000000000200000,-8.66025403784438550000,-9.99999999999999820000,-0.43301270189221930000,-0.24999999999999994000,0.86602540378443871000,
			-25.00000000000000000000,8.66025403784438910000,-9.99999999999999820000,-0.43301270189221930000,-0.24999999999999994000,0.86602540378443871000,
			2.49999999999999820000,-21.65063509461096600000,-4.99999999999999910000,-0.43301270189221930000,-0.24999999999999994000,0.86602540378443871000,
			-17.50000000000000000000,12.99038105676658200000,-4.99999999999999910000,-0.43301270189221930000,-0.24999999999999994000,0.86602540378443871000,
			9.99999999999999820000,-17.32050807568877500000,0.00000000000000000000,-0.43301270189221930000,-0.24999999999999994000,0.86602540378443871000,
			0.00000000000000000000,0.00000000000000000000,0.00000000000000000000,-0.43301270189221930000,-0.24999999999999994000,0.86602540378443871000,
			-9.99999999999999820000,17.32050807568877500000,0.00000000000000000000,-0.43301270189221930000,-0.24999999999999994000,0.86602540378443871000,
			17.50000000000000000000,-12.99038105676658200000,4.99999999999999910000,-0.43301270189221930000,-0.24999999999999994000,0.86602540378443871000,
			-2.49999999999999820000,21.65063509461096600000,4.99999999999999910000,-0.43301270189221930000,-0.24999999999999994000,0.86602540378443871000,
			25.00000000000000000000,-8.66025403784438910000,9.99999999999999820000,-0.43301270189221930000,-0.24999999999999994000,0.86602540378443871000,
			15.00000000000000200000,8.66025403784438550000,9.99999999999999820000,-0.43301270189221930000,-0.24999999999999994000,0.86602540378443871000,
			5.00000000000000270000,25.98076211353316000000,9.99999999999999820000,-0.43301270189221930000,-0.24999999999999994000,0.86602540378443871000,};
			var subdiscr1 = 1;
			var discr1 = 2;

			var discr3 = 2;
			var subdiscr1_shell = 2;
			var discr1_shell = 1;
			mpgp = GetReferenceKanonikhGewmetriaRveExampleParameters(subdiscr1, discr1, discr3, subdiscr1_shell, discr1_shell);
			mp = mpgp.Item1;
			gp = mpgp.Item2; gp.elem1 = 2; gp.elem2 = 1;

			var model_o_x_parameteroi = new o_x_parameters[graphene_sheets_number];
			var ekk_xyz = new double[graphene_sheets_number][];

			var model = new Model();
			model.SubdomainsDictionary.Add(subdomainID, new Subdomain(subdomainID));

			Dq = new double[9, 3 * (((mp.hexa1 + 1) * (mp.hexa2 + 1) * (mp.hexa3 + 1)) - ((mp.hexa1 - 1) * (mp.hexa2 - 1) * (mp.hexa3 - 1)))];
			HexaElementsOnlyRVEwithRenumbering(model, mp, Dq, renumbering_vector);

			var hexaElementsNumber = model.ElementsDictionary.Count();

			var hostGroup = model.ElementsDictionary.Where(x => (x.Key < hexaElementsNumber + 1)).Select(kv => kv.Value);
			var EmbeddedElementsIDs = new List<int>();
			int element_counter_after_Adding_sheet;
			element_counter_after_Adding_sheet = hexaElementsNumber;
			int shellElementsNumber;

			for (var i = 0; i < graphene_sheets_number; i++)
			{
				AddGrapheneSheet_with_o_x_Input_withRenumbering(model, gp, ekk_xyz[i], model_o_x_parameteroi[i], renumbering_vector, o_xsunol[i]);
				shellElementsNumber = (model.ElementsDictionary.Count() - element_counter_after_Adding_sheet) / 3;

				for (var j = shellElementsNumber + element_counter_after_Adding_sheet + 1; j < model.ElementsDictionary.Count() + 1; j++)
				{
					EmbeddedElementsIDs.Add(model.ElementsDictionary[j].ID);
				}
				element_counter_after_Adding_sheet = model.ElementsDictionary.Count();
			}

			AddLoadsOnRveFromFile_withRenumbering(model, mp.hexa1, mp.hexa2, mp.hexa3, Fxk_p_komvoi_rve, renumbering_vector);
			AddConstraintsForNonSingularStiffnessMatrix_withRenumbering(model, mp.hexa1, mp.hexa2, mp.hexa3, renumbering_vector);

			var EmbElementsIds = EmbeddedElementsIDs.ToArray();
			var embdeddedGroup = model.ElementsDictionary.Where(x => (Array.IndexOf(EmbElementsIds, x.Key) > -1)).Select(kv => kv.Value);
			var embeddedGrouping = new EmbeddedCohesiveGrouping(model, hostGroup, embdeddedGroup);

			return model;
		}


		private static Tuple<rveMatrixParameters, grapheneSheetParameters> GetReferenceKanonikhGewmetriaRveExampleParameters(int subdiscr1, int discr1, int discr3, int subdiscr1_shell, int discr1_shell)
		{
			rveMatrixParameters mp;
			mp = new rveMatrixParameters()
			{
				E_disp = 3.5, //Gpa
				ni_disp = 0.4, // stather Poisson
				L01 = 95, //150, // diastaseis
				L02 = 95, //150,
				L03 = 95, //40,
				hexa1 = discr1 * subdiscr1,// diakritopoihsh
				hexa2 = discr1 * subdiscr1,
				hexa3 = discr3
			};

			grapheneSheetParameters gp;
			gp = new grapheneSheetParameters()
			{
				// parametroi shell
				E_shell = 27196.4146610211, // GPa = 1000Mpa = 1000N / mm2
				ni_shell = 0.0607, // stathera poisson
				elem1 = discr1_shell * subdiscr1_shell,
				elem2 = discr1_shell * subdiscr1_shell,
				L1 = 50,// nm  // DIORTHOSI 2 graphene sheets
				L2 = 50,// nm
				L3 = 112.5096153846, // nm
				a1_shell = 0, // nm
				tk = 0.0125016478913782,  // 0.0125016478913782nm //0.125*40,

				//parametroi cohesive epifaneias
				T_o_3 = 0.05,// Gpa = 1000Mpa = 1000N / mm2
				D_o_3 = 0.5, // nm
				D_f_3 = 4, // nm
				T_o_1 = 0.05,// Gpa
				D_o_1 = 0.5, // nm
				D_f_1 = 4, // nm
				n_curve = 1.4
			};

			return new Tuple<rveMatrixParameters, grapheneSheetParameters>(mp, gp);
		}



		private static void HexaElementsOnlyRVEwithRenumbering(Model model, rveMatrixParameters mp, double[,] Dq, int[] renumberingVector)
		{
			// Perioxh renumbering initialization 
			renumbering renumbering = new renumbering(renumberingVector);
			// perioxh renumbering initialization ews edw 

			// Perioxh parametroi Rve Matrix
			double E_disp = mp.E_disp; //Gpa
			double ni_disp = mp.ni_disp; // stather Poisson
			double L01 = mp.L01; // diastaseis
			double L02 = mp.L02;
			double L03 = mp.L03;
			int hexa1 = mp.hexa1;// diakritopoihsh
			int hexa2 = mp.hexa2;
			int hexa3 = mp.hexa3;
			// Perioxh parametroi Rve Matrix ews edw


			// Perioxh Gewmetria shmeiwn
			int nodeCounter = 0;

			int nodeID;
			double nodeCoordX;
			double nodeCoordY;
			double nodeCoordZ;
			int kuvos = (hexa1 - 1) * (hexa2 - 1) * (hexa3 - 1);
			int endiam_plaka = 2 * (hexa1 + 1) + 2 * (hexa2 - 1);
			int katw_plaka = (hexa1 + 1) * (hexa2 + 1);

			for (var h1 = 0; h1 < hexa1 + 1; h1++)
			{
				for (var h2 = 0; h2 < hexa2 + 1; h2++)
				{
					for (var h3 = 0; h3 < hexa3 + 1; h3++)
					{
						nodeID = renumbering.GetNewNodeNumbering(Topol_rve(h1 + 1, h2 + 1, h3 + 1, hexa1, hexa2, hexa3, kuvos, endiam_plaka, katw_plaka)); // h1+1 dioti h1 einai zero based
						nodeCoordX = -0.5 * L01 + (h1 + 1 - 1) * (L01 / hexa1);  // h1+1 dioti h1 einai zero based
						nodeCoordY = -0.5 * L02 + (h2 + 1 - 1) * (L02 / hexa2);
						nodeCoordZ = -0.5 * L03 + (h3 + 1 - 1) * (L03 / hexa3);

						model.NodesDictionary.Add(nodeID, new Node(id: nodeID, x: nodeCoordX, y: nodeCoordY, z: nodeCoordZ));
						nodeCounter++;
					}
				}
			}

			var elementCounter = 0;

			var material1 = new ElasticMaterial3D(E_disp, ni_disp);

			int ElementID;
			var globalNodeIDforlocalNode_i = new int[8];

			for (var h1 = 0; h1 < hexa1; h1++)
			{
				for (var h2 = 0; h2 < hexa2; h2++)
				{
					for (var h3 = 0; h3 < hexa3; h3++)
					{
						ElementID = h1 + 1 + (h2 + 1 - 1) * hexa1 + (h3 + 1 - 1) * (hexa1) * hexa2; // h1+1 dioti h1 einai zero based
						globalNodeIDforlocalNode_i[0] = renumbering.GetNewNodeNumbering(Topol_rve(h1 + 1 + 1, h2 + 1 + 1, h3 + 1 + 1, hexa1, hexa2, hexa3, kuvos, endiam_plaka, katw_plaka));
						globalNodeIDforlocalNode_i[1] = renumbering.GetNewNodeNumbering(Topol_rve(h1 + 1, h2 + 1 + 1, h3 + 1 + 1, hexa1, hexa2, hexa3, kuvos, endiam_plaka, katw_plaka));
						globalNodeIDforlocalNode_i[2] = renumbering.GetNewNodeNumbering(Topol_rve(h1 + 1, h2 + 1, h3 + 1 + 1, hexa1, hexa2, hexa3, kuvos, endiam_plaka, katw_plaka));
						globalNodeIDforlocalNode_i[3] = renumbering.GetNewNodeNumbering(Topol_rve(h1 + 1 + 1, h2 + 1, h3 + 1 + 1, hexa1, hexa2, hexa3, kuvos, endiam_plaka, katw_plaka));
						globalNodeIDforlocalNode_i[4] = renumbering.GetNewNodeNumbering(Topol_rve(h1 + 1 + 1, h2 + 1 + 1, h3 + 1, hexa1, hexa2, hexa3, kuvos, endiam_plaka, katw_plaka));
						globalNodeIDforlocalNode_i[5] = renumbering.GetNewNodeNumbering(Topol_rve(h1 + 1, h2 + 1 + 1, h3 + 1, hexa1, hexa2, hexa3, kuvos, endiam_plaka, katw_plaka));
						globalNodeIDforlocalNode_i[6] = renumbering.GetNewNodeNumbering(Topol_rve(h1 + 1, h2 + 1, h3 + 1, hexa1, hexa2, hexa3, kuvos, endiam_plaka, katw_plaka));
						globalNodeIDforlocalNode_i[7] = renumbering.GetNewNodeNumbering(Topol_rve(h1 + 1 + 1, h2 + 1, h3 + 1, hexa1, hexa2, hexa3, kuvos, endiam_plaka, katw_plaka));

						List<Node> nodeSet = new List<Node>(8);
						for (var j = 0; j < 8; j++)
						{
							var nodeID1 = globalNodeIDforlocalNode_i[j];
							nodeSet.Add((Node)model.NodesDictionary[nodeID1]);
						}
						IReadOnlyList<INode> nodelist = new List<INode>()
						{
							model.NodesDictionary[globalNodeIDforlocalNode_i[0]],
							model.NodesDictionary[globalNodeIDforlocalNode_i[1]],
							model.NodesDictionary[globalNodeIDforlocalNode_i[2]],
							model.NodesDictionary[globalNodeIDforlocalNode_i[3]],
							model.NodesDictionary[globalNodeIDforlocalNode_i[4]],
							model.NodesDictionary[globalNodeIDforlocalNode_i[5]],
							model.NodesDictionary[globalNodeIDforlocalNode_i[6]],
							model.NodesDictionary[globalNodeIDforlocalNode_i[7]],
						};
						var e1 = new Hexa8NonLinear(nodelist, material1, GaussLegendre3D.GetQuadratureWithOrder(3, 3, 3))
						{
							ID = ElementID
						};

						model.ElementsDictionary.Add(e1.ID, e1);
						model.SubdomainsDictionary[subdomainID].Elements.Add(e1);
						elementCounter++;
					}
				}
			}

			var komvoi_rve = (hexa1 + 1) * (hexa2 + 1) * (hexa3 + 1);
			var f_komvoi_rve = kuvos;
			var p_komvoi_rve = komvoi_rve - f_komvoi_rve;
			int komvos;
			//Dq = new double[9, 3 * p_komvoi_rve];
			for (var i = 0; i < p_komvoi_rve; i++)
			{
				komvos = renumbering.GetNewNodeNumbering(f_komvoi_rve + i + 1);
				Dq[0, 3 * i + 0] = model.NodesDictionary[komvos].X;
				Dq[1, 3 * i + 1] = model.NodesDictionary[komvos].Y;
				Dq[2, 3 * i + 2] = model.NodesDictionary[komvos].Z;
				Dq[3, 3 * i + 0] = model.NodesDictionary[komvos].Y;
				Dq[4, 3 * i + 1] = model.NodesDictionary[komvos].Z;
				Dq[5, 3 * i + 2] = model.NodesDictionary[komvos].X;
				Dq[6, 3 * i + 0] = model.NodesDictionary[komvos].Z;
				Dq[7, 3 * i + 1] = model.NodesDictionary[komvos].X;
				Dq[8, 3 * i + 2] = model.NodesDictionary[komvos].Y;
			}


		}

		private static void AddGrapheneSheet_with_o_x_Input_withRenumbering(Model model, grapheneSheetParameters gp, double[] ekk_xyz, o_x_parameters o_x_parameters, int[] renumberingVector, double[] o_xsunol)
		{
			var renumbering = new renumbering(renumberingVector);

			// Perioxh parametroi Graphene sheet
			// parametroi shell
			var E_shell = gp.E_shell; // GPa = 1000Mpa = 1000N / mm2
			var ni_shell = gp.ni_shell; // stathera poisson
			var elem1 = gp.elem1;
			var elem2 = gp.elem2;
			var tk = gp.tk;  // 0.0125016478913782nm

			//parametroi cohesive epifaneias
			//T_o_3, D_o_3,D_f_3,T_o_1,D_o_1,D_f_1,n_curve
			var T_o_3 = gp.T_o_3;// Gpa = 1000Mpa = 1000N / mm2
			var D_o_3 = gp.D_o_3; // nm
			var D_f_3 = gp.D_f_3; // nm

			var T_o_1 = gp.T_o_1;// Gpa
			var D_o_1 = gp.D_o_1; // nm
			var D_f_1 = gp.D_f_1; // nm

			var n_curve = gp.n_curve;
			// Perioxh parametroi Graphene sheet ews edw


			var eswterikosNodeCounter = 0;
			var eswterikosElementCounter = 0;
			var PreviousElementsNumberValue = model.ElementsDictionary.Count();
			var PreviousNodesNumberValue = model.NodesDictionary.Count();


			// Perioxh gewmetrias (orismos nodes) meshs epifaneias
			int NodeID;
			double nodeCoordX;
			double nodeCoordY;
			double nodeCoordZ;

			for (var nNode = 0; nNode < o_xsunol.GetLength(0) / 6; nNode++)
			{
				NodeID = renumbering.GetNewNodeNumbering(eswterikosNodeCounter + PreviousNodesNumberValue + 1);
				nodeCoordX = o_xsunol[6 * nNode + 0];
				nodeCoordY = o_xsunol[6 * nNode + 1];
				nodeCoordZ = o_xsunol[6 * nNode + 2];

				model.NodesDictionary.Add(NodeID, new Node(id: NodeID, x: nodeCoordX, y: nodeCoordY, z: nodeCoordZ));
				eswterikosNodeCounter++;
			}
			var arithmosShmeiwnShellMidsurface = eswterikosNodeCounter;

			var material2 = new ShellElasticMaterial3D(E_shell, ni_shell, 5 / 6d);

			var elements = elem1 * elem2;
			var fdof_8 = 5 * (elem1 * (3 * elem2 + 2) + 2 * elem2 + 1);
			var komvoi_8 = fdof_8 / 5;
			int[,] t_shell;
			t_shell = topologia_shell_coh(elements, elem1, elem2, komvoi_8);

			var Tk_vec = new double[8];
			var VH = new double[8][];
			var midsurfaceNodeIDforlocalShellNode_i = new int[8];

			int ElementID;

			for (var j = 0; j < 8; j++) // paxos idio gia ola telements
			{
				Tk_vec[j] = tk;
			}

			for (var nElement = 0; nElement < elements; nElement++)
			{
				ElementID = eswterikosElementCounter + PreviousElementsNumberValue + 1;
				// ta dianusmata katefthunshs allazoun analoga to element 
				for (var j1 = 0; j1 < 8; j1++)
				{
					midsurfaceNodeIDforlocalShellNode_i[j1] = t_shell[nElement, j1];
					VH[j1] = new double[3];
					VH[j1][0] = o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[j1] - 1) + 3];
					VH[j1][1] = o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[j1] - 1) + 4];
					VH[j1][2] = o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[j1] - 1) + 5];
				}
				IReadOnlyList<INode> nodelist = new List<INode>()
				{
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalShellNode_i[0] + PreviousNodesNumberValue)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalShellNode_i[1] + PreviousNodesNumberValue)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalShellNode_i[2] + PreviousNodesNumberValue)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalShellNode_i[3] + PreviousNodesNumberValue)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalShellNode_i[4] + PreviousNodesNumberValue)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalShellNode_i[5] + PreviousNodesNumberValue)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalShellNode_i[6] + PreviousNodesNumberValue)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalShellNode_i[7] + PreviousNodesNumberValue)]
				};
				var e2 = new Shell8NonLinear(nodelist, material2, GaussLegendre3D.GetQuadratureWithOrder(3, 3, 3))
				{
					ID = ElementID,

					oVn_i = new double[][] { new double[] { o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[0] - 1) + 3], o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[0] - 1) + 4],o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[0] - 1) + 5] },
												 new double[] { o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[1] - 1) + 3], o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[1] - 1) + 4],o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[1] - 1) + 5] },
												 new double[] { o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[2] - 1) + 3], o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[2] - 1) + 4],o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[2] - 1) + 5] },
												 new double[] { o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[3] - 1) + 3], o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[3] - 1) + 4],o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[3] - 1) + 5] },
												 new double[] { o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[4] - 1) + 3], o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[4] - 1) + 4],o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[4] - 1) + 5] },
												 new double[] { o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[5] - 1) + 3], o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[5] - 1) + 4],o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[5] - 1) + 5] },
												 new double[] { o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[6] - 1) + 3], o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[6] - 1) + 4],o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[6] - 1) + 5] },
												 new double[] { o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[7] - 1) + 3], o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[7] - 1) + 4],o_xsunol[6 * (midsurfaceNodeIDforlocalShellNode_i[7] - 1) + 5] },},
					tk = Tk_vec
				};

				model.ElementsDictionary.Add(e2.ID, e2);
				model.SubdomainsDictionary[subdomainID].Elements.Add(e2);
				eswterikosElementCounter++;
			}

			// orismos shmeiwn katw strwshs
			for (var nNode = 0; nNode < o_xsunol.GetLength(0) / 6; nNode++)
			{
				NodeID = renumbering.GetNewNodeNumbering(eswterikosNodeCounter + PreviousNodesNumberValue + 1);
				nodeCoordX = o_xsunol[6 * nNode + 0] - 0.5 * tk * o_xsunol[6 * nNode + 3];
				nodeCoordY = o_xsunol[6 * nNode + 1] - 0.5 * tk * o_xsunol[6 * nNode + 4];
				nodeCoordZ = o_xsunol[6 * nNode + 2] - 0.5 * tk * o_xsunol[6 * nNode + 5];

				model.NodesDictionary.Add(NodeID, new Node(id: NodeID, x: nodeCoordX, y: nodeCoordY, z: nodeCoordZ));
				eswterikosNodeCounter++;
			}
			//

			//orismos elements katw strwshs
			var material3 = new BenzeggaghKenaneCohesiveMaterial(T_o_3, D_o_3, D_f_3, T_o_1, D_o_1, D_f_1, n_curve);

			var midsurfaceNodeIDforlocalCohesiveNode_i = new int[8];
			for (var nElement = 0; nElement < elements; nElement++)
			{
				ElementID = eswterikosElementCounter + PreviousElementsNumberValue + 1;
				// ta dianusmata katefthunshs allazoun analoga to element 
				for (var j1 = 0; j1 < 8; j1++)
				{
					midsurfaceNodeIDforlocalCohesiveNode_i[j1] = t_shell[nElement, j1];
					VH[j1] = new double[3];
					VH[j1][0] = o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[j1] - 1) + 3];
					VH[j1][1] = o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[j1] - 1) + 4];
					VH[j1][2] = o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[j1] - 1) + 5];
				}

				IReadOnlyList<INode> nodelist = new List<INode>()
				{
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[0] + PreviousNodesNumberValue)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[1] + PreviousNodesNumberValue)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[2] + PreviousNodesNumberValue)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[3] + PreviousNodesNumberValue)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[4] + PreviousNodesNumberValue)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[5] + PreviousNodesNumberValue)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[6] + PreviousNodesNumberValue)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[7] + PreviousNodesNumberValue)],

					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[0] + PreviousNodesNumberValue + arithmosShmeiwnShellMidsurface)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[1] + PreviousNodesNumberValue + arithmosShmeiwnShellMidsurface)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[2] + PreviousNodesNumberValue + arithmosShmeiwnShellMidsurface)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[3] + PreviousNodesNumberValue + arithmosShmeiwnShellMidsurface)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[4] + PreviousNodesNumberValue + arithmosShmeiwnShellMidsurface)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[5] + PreviousNodesNumberValue + arithmosShmeiwnShellMidsurface)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[6] + PreviousNodesNumberValue + arithmosShmeiwnShellMidsurface)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[7] + PreviousNodesNumberValue + arithmosShmeiwnShellMidsurface)]
				};

				IElementType e2 = new CohesiveShell8ToHexa20(nodelist, material3, GaussLegendre2D.GetQuadratureWithOrder(3, 3))
				{
					ID = ElementID,
					oVn_i = new double[][] { new double[] { o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[0] - 1) + 3], o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[0] - 1) + 4],o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[0] - 1) + 5] },
												new double[] { o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[1] - 1) + 3], o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[1] - 1) + 4],o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[1] - 1) + 5] },
												new double[] { o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[2] - 1) + 3], o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[2] - 1) + 4],o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[2] - 1) + 5] },
												new double[] { o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[3] - 1) + 3], o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[3] - 1) + 4],o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[3] - 1) + 5] },
												new double[] { o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[4] - 1) + 3], o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[4] - 1) + 4],o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[4] - 1) + 5] },
												new double[] { o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[5] - 1) + 3], o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[5] - 1) + 4],o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[5] - 1) + 5] },
												new double[] { o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[6] - 1) + 3], o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[6] - 1) + 4],o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[6] - 1) + 5] },
												new double[] { o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[7] - 1) + 3], o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[7] - 1) + 4],o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[7] - 1) + 5] },},
					tk = Tk_vec,
					ShellElementSide = 0
				};

				model.ElementsDictionary.Add(e2.ID, e2);
				model.SubdomainsDictionary[subdomainID].Elements.Add(e2);
				eswterikosElementCounter++;
			}

			// orismos shmeiwn anw strwshs
			for (var nNode = 0; nNode < o_xsunol.GetLength(0) / 6; nNode++)
			{
				NodeID = renumbering.GetNewNodeNumbering(eswterikosNodeCounter + PreviousNodesNumberValue + 1);
				nodeCoordX = o_xsunol[6 * nNode + 0] + 0.5 * tk * o_xsunol[6 * nNode + 3];
				nodeCoordY = o_xsunol[6 * nNode + 1] + 0.5 * tk * o_xsunol[6 * nNode + 4];
				nodeCoordZ = o_xsunol[6 * nNode + 2] + 0.5 * tk * o_xsunol[6 * nNode + 5];

				model.NodesDictionary.Add(NodeID, new Node(id: NodeID, x: nodeCoordX, y: nodeCoordY, z: nodeCoordZ));
				eswterikosNodeCounter++;
			}
			//
			//orismos elements anw strwshs 
			for (var nElement = 0; nElement < elements; nElement++)
			{
				ElementID = eswterikosElementCounter + PreviousElementsNumberValue + 1;
				// ta dianusmata katefthunshs allazoun analoga to element 
				for (var j1 = 0; j1 < 8; j1++)
				{
					midsurfaceNodeIDforlocalCohesiveNode_i[j1] = t_shell[nElement, j1];
					VH[j1] = new double[3];
					VH[j1][0] = o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[j1] - 1) + 3];
					VH[j1][1] = o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[j1] - 1) + 4];
					VH[j1][2] = o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[j1] - 1) + 5];
				}

				IReadOnlyList<INode> nodelist = new List<INode>()
				{
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[0] + PreviousNodesNumberValue)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[1] + PreviousNodesNumberValue)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[2] + PreviousNodesNumberValue)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[3] + PreviousNodesNumberValue)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[4] + PreviousNodesNumberValue)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[5] + PreviousNodesNumberValue)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[6] + PreviousNodesNumberValue)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[7] + PreviousNodesNumberValue)],

					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[0] + PreviousNodesNumberValue + 2 * arithmosShmeiwnShellMidsurface)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[1] + PreviousNodesNumberValue + 2 * arithmosShmeiwnShellMidsurface)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[2] + PreviousNodesNumberValue + 2 * arithmosShmeiwnShellMidsurface)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[3] + PreviousNodesNumberValue + 2 * arithmosShmeiwnShellMidsurface)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[4] + PreviousNodesNumberValue + 2 * arithmosShmeiwnShellMidsurface)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[5] + PreviousNodesNumberValue + 2 * arithmosShmeiwnShellMidsurface)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[6] + PreviousNodesNumberValue + 2 * arithmosShmeiwnShellMidsurface)],
					model.NodesDictionary[renumbering.GetNewNodeNumbering(midsurfaceNodeIDforlocalCohesiveNode_i[7] + PreviousNodesNumberValue + 2 * arithmosShmeiwnShellMidsurface)],
				};

				IElementType e2 = new CohesiveShell8ToHexa20(nodelist, material3, GaussLegendre2D.GetQuadratureWithOrder(3, 3))
				{
					ID = ElementID,
					oVn_i = new double[][] { new double[] { o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[0] - 1) + 3], o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[0] - 1) + 4],o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[0] - 1) + 5] },
												new double[] { o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[1] - 1) + 3], o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[1] - 1) + 4],o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[1] - 1) + 5] },
												new double[] { o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[2] - 1) + 3], o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[2] - 1) + 4],o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[2] - 1) + 5] },
												new double[] { o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[3] - 1) + 3], o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[3] - 1) + 4],o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[3] - 1) + 5] },
												new double[] { o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[4] - 1) + 3], o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[4] - 1) + 4],o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[4] - 1) + 5] },
												new double[] { o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[5] - 1) + 3], o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[5] - 1) + 4],o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[5] - 1) + 5] },
												new double[] { o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[6] - 1) + 3], o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[6] - 1) + 4],o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[6] - 1) + 5] },
												new double[] { o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[7] - 1) + 3], o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[7] - 1) + 4],o_xsunol[6 * (midsurfaceNodeIDforlocalCohesiveNode_i[7] - 1) + 5] },},
					tk = Tk_vec,
					ShellElementSide = 1
				};

				model.ElementsDictionary.Add(e2.ID, e2);
				model.SubdomainsDictionary[subdomainID].Elements.Add(e2);
				eswterikosElementCounter++;
			}
		}

		private static void AddLoadsOnRveFromFile_withRenumbering(Model model, int hexa1, int hexa2, int hexa3, double[] Fxk_p_komvoi_rve, int[] renumberingVector)
		{
			var renumbering = new renumbering(renumberingVector);

			var kuvos = (hexa1 - 1) * (hexa2 - 1) * (hexa3 - 1);

			var komvoi_rve = (hexa1 + 1) * (hexa2 + 1) * (hexa3 + 1);
			var f_komvoi_rve = kuvos;
			var p_komvoi_rve = komvoi_rve - f_komvoi_rve;
			int komvos;

			var loads = new List<INodalLoadBoundaryCondition>();
			for (var j = 0; j < p_komvoi_rve; j++)
			{
				komvos = f_komvoi_rve + j + 1;
				loads.Add(new NodalLoad
				(
					model.NodesDictionary[renumbering.GetNewNodeNumbering(komvos)],
					StructuralDof.TranslationX,
					Fxk_p_komvoi_rve[3 * (j) + 0]
				));

				loads.Add(new NodalLoad
				(
					model.NodesDictionary[renumbering.GetNewNodeNumbering(komvos)],
					StructuralDof.TranslationY,
					Fxk_p_komvoi_rve[3 * (j) + 1]
				));

				loads.Add(new NodalLoad
				(
					model.NodesDictionary[renumbering.GetNewNodeNumbering(komvos)],
					StructuralDof.TranslationZ,
					Fxk_p_komvoi_rve[3 * (j) + 2]
				));
			}

			// Afairesh fortiwn apo tous desmevmenous vathmous eleftherias 
			int nodeID;
			var supportedDOFs = new int[9];
			var endiam_plaka = 2 * (hexa1 + 1) + 2 * (hexa2 - 1);
			var katw_plaka = (hexa1 + 1) * (hexa2 + 1);
			nodeID = Topol_rve(1, 1, 1, hexa1, hexa2, hexa3, kuvos, endiam_plaka, katw_plaka);
			supportedDOFs[0] = 3 * (nodeID - f_komvoi_rve - 1) + 0;
			supportedDOFs[1] = 3 * (nodeID - f_komvoi_rve - 1) + 1;
			supportedDOFs[2] = 3 * (nodeID - f_komvoi_rve - 1) + 2;

			nodeID = Topol_rve(hexa1 + 1, 1, 1, hexa1, hexa2, hexa3, kuvos, endiam_plaka, katw_plaka);
			supportedDOFs[3] = 3 * (nodeID - f_komvoi_rve - 1) + 1;
			supportedDOFs[4] = 3 * (nodeID - f_komvoi_rve - 1) + 2;

			nodeID = Topol_rve(1, hexa2 + 1, 1, hexa1, hexa2, hexa3, kuvos, endiam_plaka, katw_plaka);
			supportedDOFs[5] = 3 * (nodeID - f_komvoi_rve - 1) + 0;
			supportedDOFs[6] = 3 * (nodeID - f_komvoi_rve - 1) + 2;

			nodeID = Topol_rve(1, 1, hexa3 + 1, hexa1, hexa2, hexa3, kuvos, endiam_plaka, katw_plaka);
			supportedDOFs[7] = 3 * (nodeID - f_komvoi_rve - 1) + 0;
			supportedDOFs[8] = 3 * (nodeID - f_komvoi_rve - 1) + 1;

			for (var j = 0; j < 9; j++)
			{
				loads.RemoveAt(supportedDOFs[8 - j]); // afairoume apo pisw pros ta mpros gia na mh xalaei h thesh twn epomenwn pou tha afairethoun
			}

			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(new List<INodalDisplacementBoundaryCondition>(), loads));
		}

		private static void AddConstraintsForNonSingularStiffnessMatrix_withRenumbering(Model model, int hexa1, int hexa2, int hexa3, int[] renumberingVector)
		{
			var renumbering = new renumbering(renumberingVector);

			var kuvos = (hexa1 - 1) * (hexa2 - 1) * (hexa3 - 1);
			var endiam_plaka = 2 * (hexa1 + 1) + 2 * (hexa2 - 1);
			var katw_plaka = (hexa1 + 1) * (hexa2 + 1);
			int nodeID;

			var constraints = new List<INodalDisplacementBoundaryCondition>();
			nodeID = renumbering.GetNewNodeNumbering(Topol_rve(1, 1, 1, hexa1, hexa2, hexa3, kuvos, endiam_plaka, katw_plaka));
			constraints.Add(new NodalDisplacement(model.NodesDictionary[nodeID], StructuralDof.TranslationX, 0d));
			constraints.Add(new NodalDisplacement(model.NodesDictionary[nodeID], StructuralDof.TranslationY, 0d));
			constraints.Add(new NodalDisplacement(model.NodesDictionary[nodeID], StructuralDof.TranslationZ, 0d));

			nodeID = renumbering.GetNewNodeNumbering(Topol_rve(hexa1 + 1, 1, 1, hexa1, hexa2, hexa3, kuvos, endiam_plaka, katw_plaka));
			constraints.Add(new NodalDisplacement(model.NodesDictionary[nodeID], StructuralDof.TranslationY, 0d));
			constraints.Add(new NodalDisplacement(model.NodesDictionary[nodeID], StructuralDof.TranslationZ, 0d));

			nodeID = renumbering.GetNewNodeNumbering(Topol_rve(1, hexa2 + 1, 1, hexa1, hexa2, hexa3, kuvos, endiam_plaka, katw_plaka));
			constraints.Add(new NodalDisplacement(model.NodesDictionary[nodeID], StructuralDof.TranslationX, 0d));
			constraints.Add(new NodalDisplacement(model.NodesDictionary[nodeID], StructuralDof.TranslationZ, 0d));

			nodeID = renumbering.GetNewNodeNumbering(Topol_rve(1, 1, hexa3 + 1, hexa1, hexa2, hexa3, kuvos, endiam_plaka, katw_plaka));
			constraints.Add(new NodalDisplacement(model.NodesDictionary[nodeID], StructuralDof.TranslationX, 0d));
			constraints.Add(new NodalDisplacement(model.NodesDictionary[nodeID], StructuralDof.TranslationY, 0d));

			model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(constraints, new List<INodalLoadBoundaryCondition>()));
		}

		private static int Topol_rve(int h1, int h2, int h3, int hexa1, int hexa2, int hexa3, int kuvos, int endiam_plaka, int katw_plaka)
		{
			int arith;
			if (h3 == 1)
			{ arith = h1 + (h2 - 1) * (hexa1 + 1) + kuvos; }
			else
			{
				if (h3 == hexa3 + 1)
				{ arith = hexa3 * (hexa1 + 1) * (hexa2 + 1) + h1 + (h2 - 1) * (hexa1 + 1); }
				else
				{
					if (h2 == 1)
					{ arith = (h3 - 2) * endiam_plaka + kuvos + katw_plaka + h1; }
					else
					{
						if (h2 == hexa2 + 1)
						{ arith = (h3 - 2) * endiam_plaka + kuvos + katw_plaka + (hexa1 + 1) + 2 * (hexa2 - 1) + h1; }
						else
						{
							if (h1 == 1)
							{ arith = kuvos + katw_plaka + (h3 - 2) * endiam_plaka + (hexa1 + 1) + (h2 - 2) * 2 + 1; }
							else
							{
								if (h1 == hexa1 + 1)
								{ arith = kuvos + katw_plaka + (h3 - 2) * endiam_plaka + (hexa1 + 1) + (h2 - 2) * 2 + 2; }
								else
								{ arith = (h1 - 1) + (h2 - 2) * (hexa1 - 1) + (h3 - 2) * (hexa1 - 1) * (hexa2 - 1); }
							}
						}
					}

				}
			}
			return arith;
		}

		private static int[,] topologia_shell_coh(int elements, int elem1, int elem2, object komvoi_8)
		{
			int elem;
			int[,] t_shell = new int[elements, 8];
			for (int nrow = 0; nrow < elem1; nrow++)
			{
				for (int nline = 0; nline < elem2; nline++)
				{
					elem = (nrow + 1 - 1) * elem2 + nline + 1;//nrow+ 1 nline+1 einai zero based 
					t_shell[elem - 1, -1 + 1] = (nrow + 1) * (3 * elem2 + 2) + (nline + 1 - 1) * 2 + 3;
					t_shell[elem - 1, -1 + 8] = (nrow + 1) * (3 * elem2 + 2) + (nline + 1 - 1) * 2 + 2;
					t_shell[elem - 1, -1 + 4] = (nrow + 1) * (3 * elem2 + 2) + (nline + 1 - 1) * 2 + 1;

					t_shell[elem - 1, -1 + 5] = (nrow + 1 - 1) * (3 * elem2 + 2) + 2 * elem2 + 1 + (nline + 1 - 1) * 1 + 2;
					t_shell[elem - 1, -1 + 7] = (nrow + 1 - 1) * (3 * elem2 + 2) + 2 * elem2 + 1 + (nline + 1 - 1) * 1 + 1;

					t_shell[elem - 1, -1 + 2] = (nrow + 1 - 1) * (3 * elem2 + 2) + (nline + 1 - 1) * 2 + 3;
					t_shell[elem - 1, -1 + 6] = (nrow + 1 - 1) * (3 * elem2 + 2) + (nline + 1 - 1) * 2 + 2;
					t_shell[elem - 1, -1 + 3] = (nrow + 1 - 1) * (3 * elem2 + 2) + (nline + 1 - 1) * 2 + 1;
				}
			}
			return t_shell;

		}

		public static IReadOnlyList<double[]> GetExpectedDisplacements()
		{
			var expectedDisplacements = new double[6][];

			expectedDisplacements[0] = new double[] {
				2.329148123666882600e-01, 1.745892242785442000e-04, 1.984890825094956000e-01, -1.201717100714760900e-04, -9.593766953486826400e-02 };
			expectedDisplacements[1] = new double[] {
				2.312129749268104200e-01, 1.832976227505586300e-04, 1.971828675655811200e-01, -9.890823091917878000e-05, -9.559024674611843500e-02 };
			expectedDisplacements[2] = new double[] {
				2.312128564190369100e-01, 1.833039372311667600e-04, 1.971829073503008300e-01, -9.889271565360632300e-05, -9.559022884187393100e-02 };
			expectedDisplacements[3] = new double[] {
				4.607459982538425500e-01, 3.749252050030026000e-04, 3.930823598857800000e-01, -1.772467305888356800e-04, -1.908369235967369300e-01 };
			expectedDisplacements[4] = new double[] {
				4.591105757986753100e-01, 3.830623301676184900e-04, 3.918152286679007500e-01, -1.584825922987456900e-04, -1.905002821626486100e-01 };
			expectedDisplacements[5] = new double[] {
				4.591104745433216600e-01, 3.830652621728760000e-04, 3.918153586144528200e-01, -1.584690208711596200e-04, -1.905002649158537600e-01 };

			return expectedDisplacements;
		}
	}

	class rveMatrixParameters
	{
		public double E_disp { get; set; }
		public double ni_disp { get; set; }
		public double L01 { get; set; }
		public double L02 { get; set; }
		public double L03 { get; set; }
		public int hexa1 { get; set; }
		public int hexa2 { get; set; }
		public int hexa3 { get; set; }

		public rveMatrixParameters()
		{

		}
		public rveMatrixParameters(double E_disp, double ni_disp, double L01, double L02, double L03, int hexa1, int hexa2, int hexa3)
		{
			this.E_disp = E_disp;
			this.ni_disp = ni_disp;
			this.L01 = L01;
			this.L02 = L02;
			this.L03 = L03;
			this.hexa1 = hexa1;
			this.hexa2 = hexa2;
			this.hexa3 = hexa3;
		}
	}

	class o_x_parameters
	{

		public o_x_parameters()
		{

		}
		public o_x_parameters(double E_disp, double ni_disp, double L01, double L02, double L03, int hexa1, int hexa2, int hexa3)
		{
		}
	}

	class grapheneSheetParameters
	{
		// parametroi shell
		public double E_shell; // GPa = 1000Mpa = 1000N / mm2
		public double ni_shell; // stathera poisson
		public int elem1;
		public int elem2;
		public double L1;// nm
		public double L2;// nm
		public double L3; // nm
		public double a1_shell; // nm
		public double tk;  // 0.0125016478913782nm
						   //parametroi cohesive epifaneias
		public double T_o_3;// Gpa = 1000Mpa = 1000N / mm2
		public double D_o_3; // nm
		public double D_f_3; // nm
		public double T_o_1;// Gpa
		public double D_o_1; // nm
		public double D_f_1; // nm
		public double n_curve = 1.4;

		public grapheneSheetParameters()
		{

		}
		public grapheneSheetParameters(double E_shell, double ni_shell, int elem1, int elem2, double L1, double L2, double L3, double a1_shell, double tk,
			double T_o_3, double D_o_3, double D_f_3, double T_o_1, double D_o_1, double D_f_1, double n_curve)
		{
			this.E_shell = E_shell; // GPa = 1000Mpa = 1000N / mm2
			this.ni_shell = ni_shell; // stathera poisson
			this.elem1 = elem1;
			this.elem2 = elem2;
			this.L1 = L1;// nm
			this.L2 = L2;// nm
			this.L3 = L3; // nm
			this.a1_shell = a1_shell; // nm
			this.tk = tk;  // 0.0125016478913782nm

			//parametroi cohesive epifaneias
			//T_o_3, D_o_3,D_f_3,T_o_1,D_o_1,D_f_1,n_curve
			this.T_o_3 = T_o_3;// Gpa = 1000Mpa = 1000N / mm2
			this.D_o_3 = D_o_3; // nm
			this.D_f_3 = D_f_3; // nm

			this.T_o_1 = T_o_1;// Gpa
			this.D_o_1 = D_o_1; // nm
			this.D_f_1 = D_f_1; // nm

			this.n_curve = n_curve;
		}
	}

	class renumbering
	{
		public int[] sunol_nodes_numbering { get; set; }

		public renumbering()
		{

		}
		public renumbering(int[] sunol_nodes_numbering)
		{
			this.sunol_nodes_numbering = sunol_nodes_numbering;
		}

		public int GetNewNodeNumbering(int initial_node_number)
		{
			return sunol_nodes_numbering[initial_node_number - 1];
		}

	}

}

