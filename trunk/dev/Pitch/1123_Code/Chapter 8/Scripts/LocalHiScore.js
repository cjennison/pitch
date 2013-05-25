class LocalHiScore {
	private var int_maxUser : int;
	private var int_minScore : int;
	private var as_users : UsersData[]; //To get all loader data name

	//Setting the maximum user to display on the menu 
	//Loading the user data and store it in here
	public function SetMaxUser ( maxUser : int ) : void {
		int_maxUser = maxUser;
		//Loading all the users data from the local machine
		LoadGameLocal();
	}
	
	public function LoadGameLocal () : void {
		//Creating the array of UsersData object
		as_users = new UsersData[int_maxUser];
		//Creating the array of int to store all the user scores data
		var a_scores : int[] = new int[int_maxUser];
		for (var i: int = 0; i < int_maxUser; i++) {
			//Creating the user data object, load data, and store it to the UsersData array
			var obj_user : UsersData = new UsersData();
			obj_user.LoadLocal(i);
			as_users[i] = obj_user;
			a_scores[i] = as_users[i].GetScore();
		}
		//Getting the minimum score for the save data purpose
		int_minScore = Mathf.Min(a_scores);
	}
	
	public function SaveGame (scores : int, name : String) : void {
		//Submitting the score if the score is higher than the minimum score of the database
		if (scores >= int_minScore) {
			var a_newData : Array = new Array(as_users);
			//Removing the last Array
			a_newData.Pop();
			//Creating new user and save it to array
			var obj_user : UsersData = new UsersData();
			obj_user.Init(name, scores);
			a_newData.Add(obj_user);
			//Setting JS Array back to Builtin
			as_users = a_newData.ToBuiltin(UsersData);
			//Sorting Data
			SortUser(as_users);
		}
		for (var i: int = 0; i < int_maxUser; i++) {
			as_users[i].SaveLocal(i);
		}
	}
	
	//Sorting the score from the maximum to minimum
	private function SortUser (array : UsersData[]) : void {
		for (var i : int = 0; i < array.length-1; i++) {
			for (var j : int = i+1; j < array.length; j++) {
				//If the first score is lower than second score swap the position
				if (array[i].GetScore() <= array[j].GetScore()) {
					var obj_temp : UsersData = array[i];
					array[i] = array[j];
					array[j] = obj_temp;
				}
			}
		}
	}

	public function GetNameData(index : int) : String {
		return as_users[index].GetName();
	}
	
	public function GetScoreData(index : int) : int {
		return as_users[index].GetScore();
	}
}