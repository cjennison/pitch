class UsersData {
	//Game Key - to make sure that each object has different key set
	public var keylocal : String = "ShooterLocal";
	private var s_keyScore : String = "Score";
	private var s_keyName : String = "Name";
	
	private var s_name : String;
	private var int_score : int;
	private var as_randomNames : String[] = ["Antony", "John", "Will", "Kate", "Jill"]; //To get a random name
	
	//Setting the user name and score
	public function Init(name : String, score : int) : void {
		int_score = score;
		s_name = name;
	}
	
	public function GetName() : String {
		return s_name;
	}
	
	public function GetScore() : int {
		return int_score;
	}

	//Saving Data
	public function SaveLocal (index : int) : void {
		//Saving user score
		PlayerPrefs.SetInt(keylocal + s_keyScore + index.ToString(), int_score);
		//Saving user name
		PlayerPrefs.SetString(keylocal + s_keyName + index.ToString(), s_name);
	}

	//Loading Data
	public function LoadLocal (index : int) : void {
		int_score = LoadScore(index);
		s_name = LoadName(index);
	}
	
	private function LoadScore (index : int) : int {
		//Checking to see if the value already exist
		var s_newKey : String = keylocal + s_keyScore + index.ToString();
		if (PlayerPrefs.HasKey(s_newKey)) {
			return PlayerPrefs.GetInt(keylocal + s_keyScore + index.ToString());
		} else {
			//If no key exist return 0 score
			return 0;
		}
	}
	
	private function LoadName (index : int) : String {
		//Checking to see if the value already exist
		var s_newKey : String = keylocal + s_keyName + index.ToString();
		if (PlayerPrefs.HasKey(s_newKey)) {
			return PlayerPrefs.GetString(keylocal + s_keyName + index.ToString());
		} else {
			//If no key exist return random name;
			var int_random : int = Random.Range(0, as_randomNames.length);
			return as_randomNames[int_random];
		}
	}
}
