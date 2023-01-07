using System;
using System.Collections;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using UnityEngine;
using UnityEditor;

public class RRT_Star : MonoBehaviour {

	// It was chosen to use arrays instead of lists to make the program faster
	// This however puts a limit to the resources that can be used
	// It is considered an acceptable trade-off since with the following 
	// maximum limits the program will keep going for up to 10 minutes 
	// before running out of space, which is longer than average usage time

	const int maxNodes = 100000;
	const int maxEdges = 100000;
	const int maxObs = 1000;
	const int maxNear = 10000;

	[Header("Parameters")]
	[SerializeField] private int widthSimArea;
	[SerializeField] private int heightSimArea;
	[SerializeField] private string randomSeed;
	[Range(5.0f,50.0f)]
	[SerializeField] private float maxEdgeStep;

	[Header("Debug")]
	[Range(0.0f,5.0f)]
	[SerializeField] private float timeBetwIterations;

	private System.Random rnd;
	private float[,] posNodes = new float[maxNodes,2];
	private float[,] posObs = new float[maxObs,2];
	private int[,] nodesOnEdges = new int[maxEdges,2];
	private float[] radObs = new float[maxObs];
	private float[] costNodes = new float[maxNodes];
	private int[] parentNodes = new int[maxNodes];
	private int numOfNodes = 0;
	private int numOfEdges = 0;
	private int numOfObs = 0;
	private bool muxClick = true;
	private bool startAlgorithm = false;
	

	void Start() {
		randomSeed = Time.realtimeSinceStartup.ToString();
		rnd = new System.Random(randomSeed.GetHashCode());
		StartCoroutine(CheckButtonStatus());
		StartCoroutine(CreateObstacles());
		StartCoroutine(RRTStarAlgorithm(timeBetwIterations));
	}

	IEnumerator CheckButtonStatus() {
		while (!startAlgorithm) { // No need to cjeck anymore once it's set to true
			// Update startAlgorithm from ButtonHandler
			startAlgorithm = GameObject.Find("ButtonHandler").GetComponent<ButtonHandler>().startAlgorithm;
			yield return null;
		}
	}

	IEnumerator CreateObstacles() {
		while (!startAlgorithm) {
			// This works only before the RRT_Star algorithm has started
			SceneView.duringSceneGui += OnSceneGUI;
			yield return null;
		}
	}

	void OnSceneGUI(SceneView sceneView) {
		if (startAlgorithm) return;
		float xMouse, yMouse;
		float[] centerObs = new float[2];
		float[] currPos = new float[2];
		Event e = Event.current;
		Vector3 mousePosition = e.mousePosition;
		Ray ray = HandleUtility.GUIPointToWorldRay(mousePosition);
		(xMouse, yMouse) = (ray.origin.x, ray.origin.y);
		// Press key - get obstacle center
		if (e.type == EventType.KeyDown && e.keyCode == KeyCode.Z && muxClick) {
			(posObs[numOfObs,0], posObs[numOfObs,1]) = (xMouse, yMouse);
			muxClick = false;
		}
		// Keep key pressed - get obstacle radius
		if (e.type == EventType.KeyDown && e.keyCode == KeyCode.Z && !muxClick) {
			(centerObs[0], centerObs[1]) = (posObs[numOfObs,0], posObs[numOfObs,1]);
			(currPos[0], currPos[1]) = (xMouse, yMouse);
			radObs[numOfObs] = GetDistance(centerObs, currPos);
		}
		// Release key - prepare for next obstacle
		if (e.type == EventType.KeyUp && e.keyCode == KeyCode.Z && !muxClick) {
			numOfObs++;
			muxClick = true;
		}
		// Click to place the starting node
		if (e.type == EventType.MouseDown && e.button == 0) {
			(currPos[0], currPos[1]) = (xMouse, yMouse);
			if (xMouse > -widthSimArea/2 && xMouse < widthSimArea/2 && yMouse > -heightSimArea/2 && yMouse < heightSimArea/2 && !IsInsideObstacle(currPos)) {
				(posNodes[0,0], posNodes[0,1]) = (xMouse, yMouse);
				numOfNodes = 1;
			}
		}
	}
	
	(float, float) InitRRTStar() {
		float[] pRand;
		bool checkFirst = false;
		float volCobs = 0, volCfree, gammaRRTStar, dimensions = 2, d, zeta_d, pi;
		pi = Mathf.PI;
		for (int i = 0; i < numOfObs; i++)
			volCobs += pi * Pow2(radObs[i]);
		volCfree = widthSimArea * heightSimArea - volCobs;
		zeta_d = pi;
		d = dimensions;
		gammaRRTStar = 2*Mathf.Pow(1 + 1/d, 1/d) * Mathf.Pow(volCfree/zeta_d, 1/d);
		// Graph starts with one Node
		if (numOfNodes == 1) {
			float[] pFirst = new float[2];
			(pFirst[0], pFirst[1]) = (posNodes[0,0], posNodes[0,1]);
			checkFirst = IsInsideObstacle(pFirst);
			numOfNodes = (checkFirst) ? 0 : 1;
		}
		if (numOfNodes == 0 || checkFirst) {
			// Generate a random one only if it wasn't already chosen
			do { pRand = GenerateRandomNode();
			} while (IsInsideObstacle(pRand));
			AddNewNode(pRand);
		}
		costNodes[0] = 0; // First node has null cost
		parentNodes[0] = 0; // First node is parent of itself
		return (gammaRRTStar, dimensions);
	}

	IEnumerator RRTStarAlgorithm(float sec) {
		// Loop to keep the method updated before the start
		while (!startAlgorithm)
			yield return null;
		
		float[] pRand, pNew, pNewInCfree;
		int[] iNearList;
		float costMin, costCheck, radV, gammaRRTStar, d;
		int iNew, iMin, iNearest, iNear, iOldParentNear, numOfNear;

		(gammaRRTStar, d) = InitRRTStar();

		// RRT_Star
		while (startAlgorithm) {
			// Explanation of "yield return":
			// Continues to next frame but then resumes where it stopped
			// So the WaitForSeconds can execute without blocking other
			// methods like OnDrawGizmos
			yield return new WaitForSeconds(sec);

			radV = Mathf.Min(gammaRRTStar * Mathf.Pow(Mathf.Log(numOfNodes)/numOfNodes, 1/d), maxEdgeStep);
			pRand = GenerateRandomNode();
			iNearest = GetNearestNode(pRand);
			pNew = Steer(iNearest, pRand, maxEdgeStep);
			pNewInCfree = StoppingConfiguration(iNearest, pNew);

			if (!IsInsideObstacle(pNewInCfree) && AreDifferentNodes(iNearest, pNewInCfree)) { // Sample again if the new node is the same of an old one
				iNew = AddNewNode(pNewInCfree);
				costNodes[iNew] = costNodes[iNearest] + GetDistanceNodes(iNearest, iNew);
				iMin = iNearest;
				costMin = costNodes[iNew];
				// From here on RRT becomes RRT_Star
				(iNearList, numOfNear) = GetNearNodes(iNew, radV);
				for (int i = 0; i < numOfNear; i++) {
					iNear = iNearList[i];
					costCheck = costNodes[iNear] + GetDistanceNodes(iNear, iNew);
					if (costCheck < costMin && ObstacleFree(iNear, iNew)) {
						iMin = iNear;
						costMin = costCheck;
					}
				}
				parentNodes[iNew] = iMin;
				costNodes[iNew] = costMin;
				AddNewEdge(iNew, iMin);
				// Rewiring near nodes
				for (int i = 0; i < numOfNear; i++) {
					iNear = iNearList[i];
					costCheck = costNodes[iNew] + GetDistanceNodes(iNear, iNew);
					if (costCheck < costNodes[iNear] && ObstacleFree(iNear, iNew)) {
						iOldParentNear = parentNodes[iNear];
						parentNodes[iNear] = iNew;
						costNodes[iNear] = costCheck;
						AddNewEdge(iNew, iNear);
						for (int j = 0; j < numOfEdges; j++) {
							if (CheckToRemoveEdge(j, iOldParentNear, iNear)) {
								RemoveEdge(j);
								break;
							}
						}
					}
				}
			}
		}
	}

	int GetNearestNode(float[] posNodeStart) {
		float[] posNode_i;
		float distWin = -1, d;
		int indexWin = -1; // Initialized just to trick compiler
		for (int i = 0; i < numOfNodes; i++) {
			posNode_i = GetPosNode(i);
			d = GetDistance(posNodeStart, posNode_i);
			if (d < distWin || distWin == -1) {
				distWin = d;
				indexWin = i;
			}
		}
		return indexWin;
	}

	bool ObstacleFree(int node1, int node2) {
		// Checks if edge between nodeStart and nodeNew is in Cfree
		float[] point1, point2;
		float[] posNode1 = GetPosNode(node1);
		float[] posNode2 = GetPosNode(node2);
		float delta, a, b, c, centerX, centerY, radius;
		float distRef, distN1P1, distN1P2, distN2P1, distN2P2;
		bool check1, check2;
		distRef = GetDistance(posNode1, posNode2);
		(a, b, c) = Points2Line(posNode1, posNode2);
		for (int i = 0; i < numOfObs; i++) {
			(centerX, centerY) = (posObs[i,0], posObs[i,1]);
			radius = radObs[i];
			(point1, point2, delta) = FindIntersectLineCircle(a, b, c, centerX, centerY, radius);
			distN1P1 = GetDistance(posNode1, point1);
			distN1P2 = GetDistance(posNode1, point2);
			distN2P1 = GetDistance(posNode2, point1);
			distN2P2 = GetDistance(posNode2, point2);
			check1 = distRef > distN1P1 && distRef > distN1P2;
			check2 = distRef > distN2P1 && distRef > distN2P2;
			if (delta >= 0 && check1 && check2) // Any intersection between the nodes
				return false;
		}
		return true;
	}

	float[] StoppingConfiguration(int nodeAttach, float[] posNodeNew) {
		// If nodeNew is in Cobs or if its edge would pass through Cobs,
		// generates nodeNewInCfree on the line between nodeNew and nodeAttach
		float[] point1, point2;
		float[] posNodeNewInCfree = posNodeNew;
		float[] posNodeAttach = GetPosNode(nodeAttach);
		float delta, a, b, c, centerX, centerY, radius, distRef;
		distRef = GetDistance(posNodeAttach, posNodeNew);
		(a, b, c) = Points2Line(posNodeAttach, posNodeNew);
		for (int i = 0; i < numOfObs; i++) {
			(centerX, centerY) = (posObs[i,0], posObs[i,1]);
			radius = radObs[i] + 1; // +1 to get away from the surface of the circle
			(point1, point2, delta) = FindIntersectLineCircle(a, b, c, centerX, centerY, radius);
			if (delta >= 0) { // 2 intersections
				(posNodeNewInCfree, distRef) = SubStoppingConfiguration(posNodeNewInCfree, distRef, posNodeAttach, point1);
				if (delta != 0) { // Skips if tangent (1 intersection)
					(posNodeNewInCfree, distRef) = SubStoppingConfiguration(posNodeNewInCfree, distRef, posNodeAttach, point2);
				}
			} // Does nothing if delta < 0 (no intersections)
		}
		(posNodeNewInCfree[0], posNodeNewInCfree[1]) = (Round(posNodeNewInCfree[0]), Round(posNodeNewInCfree[1]));
		return posNodeNewInCfree;
	}

	(float[], float) SubStoppingConfiguration(float[] posNodeCfreeOld, float distRef, float[] posNodeAttach, float[] point) {
		float[] posNodeNewInCfree = posNodeCfreeOld;
		float distCheck = GetDistance(posNodeAttach, point);
		if (distCheck < distRef) {
			(posNodeNewInCfree[0], posNodeNewInCfree[1]) = (point[0], point[1]);
			distRef = distCheck;
		}
		return (posNodeNewInCfree, distRef);
	}

	(float[], float[], float) FindIntersectLineCircle(float a, float b, float c, float cx, float cy, float r) {
		float[] intersect1 = new float[2];
		float[] intersect2 = new float[2];
		float x1, y1, x2, y2, A, B, C, delta;
		(x1,y1,x2,y2) = (0,0,0,0);
		if (b != 0) {
			A = Pow2(a) + Pow2(b);
			B = 2*(a*c + a*b*cy - Pow2(b)*cx);
			C = Pow2(b)*Pow2(cx) + Pow2(c) + Pow2(b)*Pow2(cy) + 2*b*c*cy - Pow2(b)*Pow2(r);
			delta = Pow2(B) - 4*A*C;
			if (delta >= 0) {
				x1 = (- B + Sqrt(delta)) / (2*A);
				y1 = -(a*x1 + c) / b;
				if (delta != 0) {
					x2 = (- B - Sqrt(delta)) / (2*A);
					y2 = -(a*x2 + c) / b;
				}
			}
		} else { // Vertical line
			A = 1;
			B = -2*cy;
			C = Pow2(c)/Pow2(a) + Pow2(cx) + (2*c*cx)/a + Pow2(cy) - Pow2(r);
			delta = Pow2(B) - 4*A*C;
			if (delta >= 0) {
				x1 = -c/a;
				y1 = (- B + Sqrt(delta)) / (2*A);
				if (delta != 0) {
					x2 = -c/a;
					y2 = (- B - Sqrt(delta)) / (2*A);
				}
			}
		}
		(intersect1[0], intersect1[1]) = (x1, y1);
		(intersect2[0], intersect2[1]) = (x2, y2);
		return (intersect1, intersect2, delta);
	}

	float[] Steer(int nodeAttach, float[] posNodeNew, float eta) {
		// Finds intersection on the line between nodeAttach (already on graph)
		// and nodeNew, and a circumference centered in nodeNew with radius eta
		// The intersection becomes the updated nodeNew (only if it's closer)
		float[] posNodeAttach = GetPosNode(nodeAttach);
		if (GetDistance(posNodeAttach, posNodeNew) <= eta)
			return posNodeNew;
		float[] coordIntersect;
		float[] point1 = new float[2];
		float[] point2 = new float[2];
		float x1, y1, x2, y2, a, b, c, xAtt, yAtt;
		(xAtt, yAtt) = (posNodeAttach[0], posNodeAttach[1]);
		(a, b, c) = Points2Line(posNodeAttach, posNodeNew);
		if (b != 0) {
			float m, q, x_c, y_c, r, A, B, C;
			(m, q) = (-a/b, -c/b);
			(x_c, y_c, r) = (xAtt, yAtt, eta);
			(A, B, C) = (-2*x_c, -2*y_c, Pow2(x_c) + Pow2(y_c) - Pow2(r));
			x1 = -(A + B*m + 2*m*q + Sqrt(Pow2(A) + 2*A*B*m + 4*A*m*q + Pow2(B)*Pow2(m) - 4*B*q - 4*C*Pow2(m) - 4*Pow2(q) - 4*C))/(2*(Pow2(m) + 1));
			x2 = -(A + B*m + 2*m*q - Sqrt(Pow2(A) + 2*A*B*m + 4*A*m*q + Pow2(B)*Pow2(m) - 4*B*q - 4*C*Pow2(m) - 4*Pow2(q) - 4*C))/(2*(Pow2(m) + 1));
			y1 = q - (m*(A + B*m + 2*m*q + Sqrt(Pow2(A) + 2*A*B*m + 4*A*m*q + Pow2(B)*Pow2(m) - 4*B*q - 4*C*Pow2(m) - 4*Pow2(q) - 4*C)))/(2*(Pow2(m) + 1));
			y2 = q - (m*(A + B*m + 2*m*q - Sqrt(Pow2(A) + 2*A*B*m + 4*A*m*q + Pow2(B)*Pow2(m) - 4*B*q - 4*C*Pow2(m) - 4*Pow2(q) - 4*C)))/(2*(Pow2(m) + 1));
		} else { // Vertical case
			(x1, y1) = (xAtt, yAtt + eta);
			(x2, y2) = (xAtt, yAtt - eta);
		}
		(point1[0], point1[1]) = (x1, y1);
		(point2[0], point2[1]) = (x2, y2);
		if (GetDistance(posNodeNew, point1) < GetDistance(posNodeNew, point2))
			coordIntersect = new float[] {x1, y1};
		else
			coordIntersect = new float[] {x2, y2};
		return coordIntersect;
	}

	(int[], int) GetNearNodes(int iNew, float radius) {
		// Returns all the nodes to nodeNew nearer than radius
		int[] iNear = new int[maxNear];
		float d;
		int numOfNear = 0;
		for (int i = 0; i < numOfNodes-1; i++) { // -1 to esclude iNew (last)
			d = GetDistanceNodes(iNew, i);
			if (d < radius) {
				iNear[numOfNear] = i;
				numOfNear++;
			}
		}
		return (iNear, numOfNear);
	}

	bool CheckToRemoveEdge(int i, int iPar, int iNear) {
		bool par1, par2, near1, near2;
		par1 = nodesOnEdges[i,0] == iPar;
		par2 = nodesOnEdges[i,1] == iPar;
		near1 = nodesOnEdges[i,0] == iNear;
		near2 = nodesOnEdges[i,1] == iNear;
		return (par1 && near2 || near1 && par2) ? true : false;
	}

	bool AreDifferentNodes(int nodeNearest, float[] pNew) {
		float xNearest, yNearest, xNew, yNew;
		(xNearest, yNearest) = (posNodes[nodeNearest,0], posNodes[nodeNearest,1]);
		(xNew, yNew) = (pNew[0], pNew[1]);
		return (xNearest != xNew || yNearest != yNew) ? true : false;
	}

	bool IsInsideObstacle(float[] posNode) {
		float[] posCenter = new float[2];
		for (int i = 0; i < numOfObs; i++) {
			(posCenter[0], posCenter[1]) = (posObs[i,0], posObs[i,1]);
			if (GetDistance(posNode, posCenter) <= radObs[i]+1) {
				return true;
			}
		}
		return false;
	}

	int AddNewNode(float[] coord) {
		posNodes[numOfNodes,0] = coord[0];
		posNodes[numOfNodes,1] = coord[1];
		numOfNodes++;
		return numOfNodes-1; // Returns index of last node added
	}

	void AddNewEdge(int node1, int node2) {
		nodesOnEdges[numOfEdges,0] = node1;
		nodesOnEdges[numOfEdges,1] = node2;
		numOfEdges++;
	}

	void RemoveEdge(int i) {
		// Replaces nodes on the index to be removed with the last edge
		nodesOnEdges[i,0] = nodesOnEdges[numOfEdges-1,0];
		nodesOnEdges[i,1] = nodesOnEdges[numOfEdges-1,1];
		numOfEdges--;
	}

	float[] GenerateRandomNode() {
		float x, y;
		float[] coordNewNode;
		x = Round(rnd.Next(-widthSimArea/2, widthSimArea/2));
		y = Round(rnd.Next(-heightSimArea/2, heightSimArea/2));
		coordNewNode = new float[] {x, y};
		return coordNewNode;
	}

	(float, float, float) Points2Line(float[] point1, float[] point2) {
		// Returns a,b,c of the implicit form a*x + b*y + c = 0
		float x1,x2,y1,y2;
		(x1, y1) = (point1[0], point1[1]);
		(x2, y2) = (point2[0], point2[1]);
		if (x1 == x2 && y1 == y2)	// "Error: same point"
			return (0,1,-y1);		// Leads back to the Orizontal case
		if (x1 == x2)				// Vertical case
			return (1,0,-x1);
		float m, q;
		m = (y1-y2)/(x1-x2);
		q = y1 - m * x1;
		return (-m,1,-q);
	}

	float GetDistance(float[] point1, float[] point2) {
		float x1,x2,y1,y2;
		(x1, y1) = (point1[0], point1[1]);
		(x2, y2) = (point2[0], point2[1]);
		return (float)Sqrt(Pow2(x1 - x2) + Pow2(y1 - y2));
	}

	float GetDistanceNodes(int node1, int node2) {
		float[] pos1, pos2;
		float x1,x2,y1,y2;
		pos1 = GetPosNode(node1);
		pos2 = GetPosNode(node2);
		(x1, y1) = (pos1[0], pos1[1]);
		(x2, y2) = (pos2[0], pos2[1]);
		return (float)Sqrt(Pow2(x1 - x2) + Pow2(y1 - y2));
	}

	float[] GetPosNode(int i) {
		float[] posNode = new float[2];
		(posNode[0], posNode[1]) = (posNodes[i,0], posNodes[i,1]);
		return posNode;
	}

	float Pow2(float x) {
		return Mathf.Pow(x,2);
	}

	float Sqrt(float x) {
		return Mathf.Sqrt(x);
	}

	float Round(float x) {
		// Used to avoid coefficients too high for almost vertical edges
		// Nodes can spawn only on a 1x1 grid (integer precision)
		return Mathf.Round(x);
	}

	void OnDrawGizmos() {
		// Draw obstacles
		Vector3 v = Vector3.forward;
		Handles.color = Color.red;
		for (int i = 0; i < numOfObs; i++) {
			Vector3 posO = new Vector3(posObs[i,0], posObs[i,1], 0);
			Handles.DrawSolidDisc(posO, v, radObs[i]);
		}
		// Draw edges
		Handles.color = Color.white;
		for (int i = 0; i < numOfEdges; i++) {
			int index1 = nodesOnEdges[i,0];
			int index2 = nodesOnEdges[i,1];
			Vector3 pos1 = new Vector3(posNodes[index1,0], posNodes[index1,1], 0);
			Vector3 pos2 = new Vector3(posNodes[index2,0], posNodes[index2,1], 0);
			Handles.DrawLine(pos1, pos2);
		}
		// Draw the limits of the sim area
		Vector3 edge1 = new Vector3(-widthSimArea/2, -heightSimArea/2, 0);
		Vector3 edge2 = new Vector3(+widthSimArea/2, -heightSimArea/2, 0);
		Vector3 edge3 = new Vector3(-widthSimArea/2, +heightSimArea/2, 0);
		Vector3 edge4 = new Vector3(+widthSimArea/2, +heightSimArea/2, 0);
		Handles.DrawLine(edge1, edge2);
		Handles.DrawLine(edge1, edge3);
		Handles.DrawLine(edge2, edge4);
		Handles.DrawLine(edge3, edge4);
	}
}
