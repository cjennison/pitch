public var customSkin : GUISkin;

//Setting the default string on the submit text field
public static var userName : String = "Player 1"; 

//Setting the maximum number of the users displayed on the scoreboard
public var maxUsers : int = 10; 

//Creating the enum parameter for the menu page
enum Page { GAMEOVER, LOCALSCORE, SERVERSCORE }; 

//Creating the enum parameter for the menu page
private var e_page : Page = Page.GAMEOVER; 

//Creating the scroll position for the local high score scroller area
private var scrollPositionL : Vector2 = Vector2.zero;

//Creating the scroll position for the server high score scroller area
private var scrollPositionS : Vector2 = Vector2.zero;

//Checking if the restart button is clicked by the user
private var b_isClickRestart : boolean = false; 

//Checking if the submit button is clicked by the user
private var b_isClickSubmit : boolean = false; 

private var obj_localHiScore : LocalHiScore; //Creating the LocalHiScore Object
private var obj_serverHiScore : ServerHiScore; //Creating the ServerHiScore Object

public function Start() : void {
	//Initilizing
	e_page = Page.GAMEOVER;
	scrollPosition = Vector2.zero;
	b_isClickRestart = false;
	b_isClickSubmit = false;
	
	//Creating a Local Hiscore Object
	obj_localHiScore = new LocalHiScore();
	//Setting the maximum scores to show on the table & loading the local high score data here
	obj_localHiScore.SetMaxUser(maxUsers);
	//Creating a Server Hiscore Object
	obj_serverHiScore = GetComponent.<ServerHiScore>();
}

public function OnGUI() : void {
	if (StaticVars.b_isGameOver) {
		GUI.skin = customSkin;
		//Checking if we didn't click on the restart button
		if (b_isClickRestart == false) { 
			//Checking for the current page
			switch (e_page) {
				case Page.GAMEOVER:
					GameoverPage(); //Creating game over page
					break;
				case Page.LOCALSCORE:
					LocalScorePage(); //Creating local score page
					break;
				case Page.SERVERSCORE:
					ServerScorePage(); //Creating server score page
					break;
			}
			//Creating the Restart Button
			if (GUI.Button(new Rect((Screen.width - 240)*0.5, (Screen.height*0.1) + 320, 240, 30), "RESTART")) {
				b_isClickRestart = true;
				Restart();
			}
		} else {
			//If we clicked on the restart button - just put the Loading... text here
			GUI.Box(new Rect(Screen.width*0.1, Screen.height*0.1, Screen.width * 0.8, Screen.height * 0.8), "", GUI.skin.GetStyle("Box2"));
			GUI.Label(new Rect((Screen.width-150)*0.5, (Screen.height-50)*0.5, 150, 50), "LOADING...", GUI.skin.GetStyle("Text1"));
		}
	}
}

//Creating Gameover Page GUI
private function GameoverPage() : void {
	//Creating the background box
	GUI.Box(new Rect(Screen.width*0.1, Screen.height*0.1, Screen.width * 0.8, Screen.height * 0.8), "GAMEOVER", GUI.skin.GetStyle("Box2"));
	//Creating Text Label to show the final score of the player
	GUI.Label(new Rect((Screen.width - 400)*0.5, (Screen.height*0.1) + 50, 400, 25), "Final Score: " + TimeScoreUI.int_currentScore.ToString(), GUI.skin.GetStyle("Text1"));
	//If the user didn't click submit, we create the submit button and text field for the player to submit the score
	if (b_isClickSubmit == false) {
		GUI.Label(new Rect((Screen.width - 300)*0.5, (Screen.height*0.1) + 80, 300, 25), "Enter Your Name", GUI.skin.GetStyle("Text1"));
		//Creating the input text field to get the player name
		userName = GUI.TextField(new Rect((Screen.width - 240)*0.5, (Screen.height*0.1) + 120, 240, 40), userName, 8);
		//Submit button
		if (GUI.Button(new Rect((Screen.width - 240)*0.5, (Screen.height*0.1) + 200, 240, 30), "SUBMIT")) {
			b_isClickSubmit = true;
			//TODO: Submitting both local and server high score here
			obj_localHiScore.SaveGame(TimeScoreUI.int_currentScore, userName); //Submitting to the local score
			//Submitting to server
			obj_serverHiScore.SendScore(TimeScoreUI.int_currentScore, userName);
		}
	}
	//Creating the Local Hi-Score page button 
	if (GUI.Button(new Rect((Screen.width - 240)*0.5, (Screen.height*0.1) + 240, 240, 30), "LOCAL HI-SCORE")) {
		e_page = Page.LOCALSCORE;
	}
	//Creating the Server Hi-Score page button 
	if (GUI.Button(new Rect((Screen.width - 240)*0.5, (Screen.height*0.1) + 280, 240, 30), "SERVER HI-SCORE")) {
		//TODO: Loading the score data from server here
		obj_serverHiScore.GetScores();
		e_page = Page.SERVERSCORE;
	}
} 

//Loading the local scores
private function LocalScorePage() : void {
	//Creating the background box
	GUI.Box(new Rect(Screen.width*0.1, Screen.height*0.1, Screen.width * 0.8, Screen.height * 0.8), "LOCAL HI-SCORE", GUI.skin.GetStyle("Box2"));
	//Creating the scrolled area and scrollbar to view the player scores
	scrollPositionL = GUI.BeginScrollView (new Rect ((Screen.width - 320)*0.5, (Screen.height*0.1) + 80, 320, 180), scrollPositionL, new Rect (0, 0, 300, 30*maxUsers));
	for (var i: int = 0; i < maxUsers; i++) {
		//Setting the number of the user
		GUI.Label(new Rect(0, i * 30, 35, 30), (i+1).ToString() + ".");
		//TODO: Showing the user name and score here
		GUI.Label(new Rect(35, i * 30, 120, 30), obj_localHiScore.GetNameData(i));
    	GUI.Label(new Rect(155, i * 30, 145, 30), GlobalFunction.addCommasInt(obj_localHiScore.GetScoreData(i)), GUI.skin.GetStyle("Score"));
    }
	GUI.EndScrollView (); //End Scroll Area
	if (GUI.Button(new Rect((Screen.width - 240)*0.5, (Screen.height*0.1) + 280, 240, 30), "BACK")) {
		e_page = Page.GAMEOVER;
	}
}

//Loading score from server
private function ServerScorePage() : void {
	//Creating the background box
	GUI.Box(new Rect(Screen.width*0.1, Screen.height*0.1, Screen.width * 0.8, Screen.height * 0.8), "SERVER HI-SCORE", GUI.skin.GetStyle("Box2"));
	//TODO: Checking is the loader completed
	if (obj_serverHiScore.IsLoaded()) {
		var int_numUsers : int = obj_serverHiScore.GetUserLength();
		if (int_numUsers >= maxUsers) {
			int_numUsers = maxUsers;
		}
		scrollPositionS = GUI.BeginScrollView (new Rect ((Screen.width - 320)*0.5, (Screen.height*0.1) + 80, 320, 180), scrollPositionS, new Rect (0, 0, 300, 30*int_numUsers));
		for (var i: int = 0; i < int_numUsers; i++) {
			//Setting the number of the user
	    	GUI.Label(new Rect(0, i * 30, 35, 30), (i+1).ToString() + ".");
	    	//TODO: Showing the user name and score here
	    	GUI.Label(new Rect(35, i * 30, 120, 30), obj_serverHiScore.GetNameData(i));
	    	GUI.Label(new Rect(155, i * 30, 145, 30), GlobalFunction.addCommasInt(obj_serverHiScore.GetScoreData(i)), GUI.skin.GetStyle("Score"));
	    }
		GUI.EndScrollView (); //End Scroll Area
	} else {
		//TODO: If the loader doesn't complete display Loading... text
		GUI.Label(new Rect((Screen.width-150)*0.5, (Screen.height*0.1)+120, 150, 50), "LOADING...", GUI.skin.GetStyle("Text1"));
	}
	if (GUI.Button(new Rect((Screen.width - 240)*0.5, (Screen.height*0.1) + 280, 240, 30), "BACK")) {
		e_page = Page.GAMEOVER;
	}
}


private function Restart() : IEnumerator {
	yield new WaitForSeconds (1.0); //Wait for 1.0 secs. until do the next function
	Application.LoadLevel(0);
}