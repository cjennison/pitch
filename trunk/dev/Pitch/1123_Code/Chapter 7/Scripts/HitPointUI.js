public var player : New3PSController;

public var frameTexture : Texture2D;
public var hpTexture : Texture2D;
public var aiTexture : Texture2D;
public var textHpTexture : Texture2D;
public var textAiTexture : Texture2D;

public var customSkin : GUISkin;

private var int_index : int;
private var int_range : int;
private var f_curHpPercent : float;
private var a_AIs : Array;
private var ais : AIController[];

public function Awake() : void {
	int_index = 1;
	f_curHpPercent = -0.1;
	var g_AIs : GameObject[] = GameObject.FindGameObjectsWithTag("AI");
	a_AIs = new Array();
	for (var i : int = 0; i < g_AIs.Length; i++) {
		var obj_AIController : AIController = g_AIs[i].GetComponent(AIController);
		obj_AIController.SetIndex(i+1);
		a_AIs.Push(obj_AIController);
	}
	ais = a_AIs.ToBuiltin(AIController);
	int_range = ais.Length;
}

// Update is called once per frame
public function Update() : void {
	//New
	if (player == null) {
		StaticVars.b_isGameOver = true;
	} else {
		if (int_range == 0) {
			StaticVars.b_isGameOver = true;
		}
	}
}

public function OnGUI() : void {
	GUI.skin = customSkin;
	//Draw Text
	GUI.DrawTexture (Rect (10,10,46,32), textHpTexture);
	
	//Character Hp
	// Create one Group to contain both images
	// Adjust the first 2 coordinates to place it somewhere else on-screen
	GUI.BeginGroup (Rect (110,15,156,21));
	// Draw the background image
	GUI.DrawTexture(Rect (0,0,156,21), frameTexture);
	// Create a second Group which will be clipped
	// We want to clip the image and not scale it, which is why we need the second Group
	if (player != null) {
		GUI.BeginGroup (Rect (0,0,player.GetHpPercent() * 156, 21));
	} else {
		GUI.BeginGroup (Rect (0,0,0, 21));
	}
	// Draw the foreground image
	GUI.DrawTexture (Rect (0,0,156,21), hpTexture);
	// End both Groups
	GUI.EndGroup ();
	GUI.EndGroup ();
	
	//Check AI Hit
	CheckAIHit();
	if (f_curHpPercent > 0.0) {
		GUI.DrawTexture (Rect (10,42,95,32), textAiTexture);
		
		//AI HP
		// Create one Group to contain both images
		// Adjust the first 2 coordinates to place it somewhere else on-screen
		GUI.BeginGroup (Rect (110,47,156,21));
		// Draw the background image
		GUI.DrawTexture(Rect (0,0,156,21), frameTexture);
		GUI.Label(Rect (0,0,156,21), "ENEMY " + int_index.ToString(), GUI.skin.GetStyle("CustomText2"));
		
		// Create a second Group which will be clipped
		// We want to clip the image and not scale it, which is why we need the second Group
		GUI.BeginGroup (Rect (0,0,f_curHpPercent * 156, 21));
		
		// Draw the foreground image
		GUI.DrawTexture (Rect (0,0,156,21), aiTexture);
		GUI.Label(Rect (0,0,156,21), "ENEMY " + int_index.ToString(), GUI.skin.GetStyle("CustomText2"));
		// End both Groups
		GUI.EndGroup ();
		GUI.EndGroup ();
	}
}

public function CheckAIHit() : void {
	var b_isNoHit : boolean = true;
	for (var i : int = 0; i < ais.Length; i++) {
		if (ais[i] != null) {
			if ((ais[i].GetHit()) && (ais[i].GetHpPercent() != f_curHpPercent)) {
				b_isNoHit = false;
				ais[i].SetHit(false);
				int_index = ais[i].GetIndex();
				f_curHpPercent = ais[i].GetHpPercent();
				break;
			}
		} else {
			//The AI is already distroyed, so make the percent equal 0
			//and remove the array ai
			f_curHpPercent = 0.0;
			a_AIs.RemoveAt(i);
			break;
		}
	}
	ais = a_AIs.ToBuiltin(AIController);
	int_range = ais.Length;
}