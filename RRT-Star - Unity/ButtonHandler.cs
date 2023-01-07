using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ButtonHandler : MonoBehaviour {
	
	[HideInInspector] public bool startAlgorithm = false;

	public void OnButtonPress() {
		startAlgorithm = true;
	}
}
